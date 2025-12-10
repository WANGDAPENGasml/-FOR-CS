
// --- constants ---
#define RINGLOG_SLOT_SIZE      26U     // was 24U; now 26 bytes because of 2-byte magic
#define RINGLOG_PROBE_STEPS    16U
#define RINGLOG_SAMPLE_STRIDE  64U

#define RINGLOG_FLAG_VALID     (1u << 0)
#define RINGLOG_FLAG_PENDING   (1u << 1)
#define RINGLOG_MAGIC          0x55AA  // 2-byte sentinel placed before seq

// --- record layout (26 bytes) ---
typedef struct __attribute__((packed)) {
    uint16_t magic;   // 0x55AA sentinel to avoid misjudgment at first power-up
    uint32_t seq;     // per-epoch monotonic sequence (wrap allowed)
    uint16_t epoch;   // epoch counter (incremented on ring wrap)
    uint8_t  flags;   // bit0=VALID, bit1=PENDING
    uint8_t  crc8;    // CRC8 over data[16] (or header if preferred)
    uint8_t  data[16];// application payload, fixed 16 bytes
} ringlog_record_t;   // sizeof = 26
