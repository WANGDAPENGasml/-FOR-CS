
#include "ringlog.h"
#include <string.h>

static const ringlog_storage_t* S = NULL;  // Bound storage backend
static uint32_t g_num_slots = 0;           // Total slots = size / 26 bytes

// Runtime state
static uint32_t g_wp_slot = 0;             // Write pointer (slot index)
static uint16_t g_epoch   = 0;             // Epoch counter
static uint32_t g_seq     = 0;             // Sequence counter

/*** Bind selected storage backend ***/
void ringlog_bind_storage(const ringlog_storage_t* ops) {
    S = ops;
}

/*** Ring addressing utilities ***/
static inline uint32_t slot_addr(uint32_t slot_index) {
    return S->base + slot_index * RINGLOG_SLOT_SIZE;
}
static inline uint32_t ring_step_addr(uint32_t addr, uint32_t step) {
    addr += step;
    if (addr >= S->base + S->size) addr = S->base + (addr - (S->base + S->size));
    return addr;
}
static inline uint32_t ring_step_slot(uint32_t slot_index, uint32_t step_slots) {
    uint32_t next = slot_index + step_slots;
    if (next >= g_num_slots) next -= g_num_slots;
    return next;
}

/*** Init (device + ring parameters)
 *  For MRAM (PM004MNIA), honor tPU ≈ 1.5 ms before any access; see datasheet. **/
void ringlog_init(void) {
    if (!S) return;
    if (S->init) S->init();
    g_num_slots = S->size / RINGLOG_SLOT_SIZE;
}

/*** CRC8 (poly 0x07, init 0x00) ***/
uint8_t ringlog_crc8(const uint8_t* buf, uint32_t len) {
    uint8_t crc = 0x00;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/*** Read a slot ***/
bool ringlog_read_slot(uint32_t slot_index, ringlog_record_t* out) {
    if (!S || slot_index >= g_num_slots) return false;
    uint32_t addr = slot_addr(slot_index);
    return (S->read(addr, (uint8_t*)out, sizeof(*out)) == HAL_OK);
}

/*** Lightweight validity checks ***/
static inline bool has_magic(const ringlog_record_t* r) {
    return (r->magic == RINGLOG_MAGIC);
}
static inline bool is_flag_valid(const ringlog_record_t* r) {
    return (bool)((r->flags & RINGLOG_FLAG_VALID) && !(r->flags & RINGLOG_FLAG_PENDING));
}
static inline bool is_candidate(const ringlog_record_t* r) {
    // Candidate in search: must have magic and flags show VALID (CRC can be deferred for performance)
    return has_magic(r) && is_flag_valid(r);
}
static inline bool strong_valid(const ringlog_record_t* r) {
    // Strong validity used when finalizing head: magic + flags + CRC on payload
    if (!has_magic(r) || !is_flag_valid(r)) return false;
    return (ringlog_crc8(r->data, sizeof(r->data)) == r->crc8);
}

/*** Key comparison (epoch, then seq) ***/
static inline int key_cmp(const ringlog_record_t* a, const ringlog_record_t* b) {
    if (a->epoch > b->epoch) return +1;
    if (a->epoch < b->epoch) return -1;
    if (a->seq   > b->seq)   return +1;
    if (a->seq   < b->seq)   return -1;
    return 0;
}

/*** Two-phase commit append ***/
HAL_StatusTypeDef ringlog_append(const uint8_t data[16]) {
    if (!S) return HAL_ERROR;

    // Phase 1: write full record with magic and PENDING=1, VALID=0
    ringlog_record_t r;
    r.magic = RINGLOG_MAGIC;            // sentinel to distinguish initialized slots
    r.seq   = g_seq++;
    r.epoch = g_epoch;
    r.flags = RINGLOG_FLAG_PENDING;
    r.crc8  = ringlog_crc8(data, 16);
    memcpy(r.data, data, 16);

    uint32_t addr = slot_addr(g_wp_slot);

    // Ensure slot size respects backend min burst (MRAM requires >=2 bytes) 
    if ((RINGLOG_SLOT_SIZE % S->min_write_burst) != 0) return HAL_ERROR;

    HAL_StatusTypeDef st = S->write(addr, (uint8_t*)&r, sizeof(r));
    if (st != HAL_OK) return st;

    // Phase 2: patch flags to VALID; align to backend min burst (>=2 for MRAM per datasheet)
    uint8_t patch[2] = { (uint8_t)((r.flags | RINGLOG_FLAG_VALID) & ~RINGLOG_FLAG_PENDING), 0x00 };
    uint16_t patch_len = (S->min_write_burst <= 2) ? 2 : S->min_write_burst;
    st = S->write(addr + offsetof(ringlog_record_t, flags), patch, patch_len);
    if (st != HAL_OK) return st;

    // Advance write pointer and epoch on wrap
    g_wp_slot = ring_step_slot(g_wp_slot, 1);
    if (g_wp_slot == 0) g_epoch++;

    return HAL_OK;
}

/*** Validity sampling (early-phase detection) ***/
static uint32_t count_valid_samples(void) {
    uint32_t count = 0;
    for (uint32_t i = 0; i < g_num_slots; i += RINGLOG_SAMPLE_STRIDE) {
        ringlog_record_t r;
        if (ringlog_read_slot(i, &r) && is_candidate(&r)) count++;
    }
    return count;
}

/*** Linear head search (for early non-wrapped phase with contiguous valid region) ***/
static bool linear_find_head(uint32_t* out_head) {
    uint32_t last_valid = UINT32_MAX;
    for (uint32_t i = 0; i < g_num_slots; ++i) {
        ringlog_record_t r;
        if (!ringlog_read_slot(i, &r)) break;
        if (is_candidate(&r)) last_valid = i;
        else break; // first non-candidate -> previous region assumed contiguous valid
    }
    if (last_valid == UINT32_MAX) return false;
    *out_head = last_valid;
    return true;
}

/*** Neighbor probing when mid/right non-candidate during binary search ***/
static bool probe_valid_near(uint32_t start_idx, uint32_t end_idx,
                             uint32_t* out_idx, ringlog_record_t* out_rec) {
    // Probe forward
    for (uint32_t step = 0; (start_idx + step) <= end_idx && step < RINGLOG_PROBE_STEPS; ++step) {
        uint32_t ii = start_idx + step;
        ringlog_record_t r;
        if (ringlog_read_slot(ii, &r) && is_candidate(&r)) {
            *out_idx = ii; *out_rec = r; return true;
        }
    }
    // Probe backward
    for (uint32_t step = 1; (int32_t)start_idx - (int32_t)step >= 0 && step <= RINGLOG_PROBE_STEPS; ++step) {
        uint32_t ii = start_idx - step;
        ringlog_record_t r;
        if (ringlog_read_slot(ii, &r) && is_candidate(&r)) {
            *out_idx = ii; *out_rec = r; return true;
        }
    }
    return false;
}

/*** Binary search for pivot (smallest key in rotated sorted sequence)
 *  Uses magic+flag-based candidacy to avoid misjudgment on fresh devices. ***/
static bool find_pivot(uint32_t* out_pivot) {
    ringlog_record_t leftRec, rightRec;
    bool leftOk  = ringlog_read_slot(0, &leftRec) && is_candidate(&leftRec);
    bool rightOk = ringlog_read_slot(g_num_slots - 1, &rightRec) && is_candidate(&rightRec);

    // Non-rotated or fully sorted case: pivot = 0
    if (leftOk && rightOk && (key_cmp(&leftRec, &rightRec) <= 0)) {
        *out_pivot = 0;
        return true;
    }

    uint32_t left = 0, right = g_num_slots - 1;
    while (left < right) {
        uint32_t mid = left + ((right - left) >> 1);

        ringlog_record_t midRec, rRec;
        bool midOk = ringlog_read_slot(mid, &midRec) && is_candidate(&midRec);
        bool rOk   = ringlog_read_slot(right, &rRec) && is_candidate(&rRec);

        if (!midOk) {
            uint32_t nearIdx; ringlog_record_t nearRec;
            if (probe_valid_near(mid, right, &nearIdx, &nearRec)) {
                mid = nearIdx; midRec = nearRec; midOk = true;
            } else { right = (right > 0) ? (right - 1) : 0; continue; }
        }
        if (!rOk) {
            uint32_t nearIdx; ringlog_record_t nearRec;
            if (probe_valid_near(right, g_num_slots - 1, &nearIdx, &nearRec)) {
                right = nearIdx; rRec = nearRec; rOk = true;
            } else { right = (right > 0) ? (right - 1) : 0; continue; }
        }

        // If mid > right, pivot is in right half; else in left half (including mid)
        if (key_cmp(&midRec, &rRec) > 0) left = mid + 1;
        else                             right = mid;
    }

    *out_pivot = left;
    return true;
}

/*** Cold boot recovery
 *  Final head validation uses strong_valid() (magic+flags+CRC) to avoid false positives. ***/
bool ringlog_cold_boot_recover(void) {
    ringlog_init();
    if (!S) return false;

    uint32_t samples = count_valid_samples();
    if (samples == 0) { // empty device
        g_wp_slot = 0; g_epoch = 0; g_seq = 0; return true;
    }
    // Early phase with few candidates: use linear scan for head
    if (samples <= (g_num_slots / RINGLOG_SAMPLE_STRIDE) / 8) {
        uint32_t head;
        if (!linear_find_head(&head)) return false;
        g_wp_slot = ring_step_slot(head, 1);

        ringlog_record_t r;
        if (!ringlog_read_slot(head, &r) || !strong_valid(&r)) return false;
        g_epoch = r.epoch; g_seq = r.seq + 1;
        return true;
    }

    // Rotated sorted sequence: binary search pivot
    uint32_t pivot = 0;
    if (!find_pivot(&pivot)) return false;
    uint32_t head = (pivot == 0) ? (g_num_slots - 1) : (pivot - 1);

    // Restore write pointer and counters from head (strong validation with CRC)
    g_wp_slot = ring_step_slot(head, 1);
    ringlog_record_t r;
    if (!ringlog_read_slot(head, &r) || !strong_valid(&r)) return false;
    g_epoch = r.epoch; g_seq = r.seq + 1;
    return true;
}

/*** State getters ***/
uint32_t ringlog_current_slot(void) { return g_wp_slot; }
uint16_t ringlog_current_epoch(void){ return g_epoch;   }
uint32_t ringlog_current_seq(void)  { return g_seq;     }

