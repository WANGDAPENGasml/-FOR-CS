
#ifndef RINGLOG_BACKEND_GD32_H
#define RINGLOG_BACKEND_GD32_H

#include <stdint.h>
#include <stdbool.h>
#include "ringlog.h"

// Include GD32H759 HAL headers (adjust paths to your SDK)
#include "gd32h7xx.h"
#include "gd32h7xx_i2c.h"
#include "gd32h7xx_spi.h"
#include "gd32h7xx_ospi.h"

// ================================
// Public backend instances to bind
// ================================

// I2C FRAM backend instance
extern ringlog_storage_t RING_GD32_I2C_FRAM;

// SPI FRAM backend instance
extern ringlog_storage_t RING_GD32_SPI_FRAM;

// OSPI/QSPI MRAM (PM004MNIA) backend instance
extern ringlog_storage_t RING_GD32_OSPI_MRAM_PM004;

// ================================
// Binder APIs (pass handles/config)
// ================================

/**
 * Bind GD32 I2C FRAM context.
 * @param i2c_periph   I2C peripheral base (e.g., I2C0)
 * @param fram_addr8   8-bit I2C address used by GD32 HAL (7-bit << 1)
 * @param base         base address (usually 0)
 * @param size         FRAM capacity in bytes (e.g., 256*1024)
 * @param addr16       true if device uses 16-bit memory addresses; false for 8-bit
 */
void rl_bind_gd32_i2c_fram(uint32_t i2c_periph, uint16_t fram_addr8,
                           uint32_t base, uint32_t size, bool addr16);

/**
 * Bind GD32 SPI FRAM context.
 * @param hspi         pointer to SPI HAL handle
 * @param cs_gpio      chip-select GPIO port
 * @param cs_pin       chip-select pin
 * @param base         base address (usually 0)
 * @param size         FRAM capacity in bytes
 * @param requires_wren true if FRAM requires WREN(0x06) before write (most do)
 */
void rl_bind_gd32_spi_fram(SPI_HandleTypeDef* hspi,
                           GPIO_TypeDef* cs_gpio, uint16_t cs_pin,
                           uint32_t base, uint32_t size, bool requires_wren);

/**
 * Bind GD32 OSPI/QSPI MRAM (PM004MNIA) context.
 * @param ospi_periph  OSPI peripheral base (e.g., OSPI0)
 * @param ospi_cfg     pointer to OSPI parameter/config struct
 * @param base         base address (0)
 * @param size         PM004MNIA capacity in bytes (512*1024)
 * @param use_qpi      true to operate in QPI (4-lines) after init; false for SPI (1-line)
 */
void rl_bind_gd32_ospi_mram_pm004(uint32_t ospi_periph, ospi_parameter_struct* ospi_cfg,
                                  uint32_t base, uint32_t size, bool use_qpi);

// ================================
// Optional helpers (MRAM / QPI)
// ================================

/** Enter QPI mode (PM004MNIA 0x35). */
bool rl_mram_enter_qpi(void);

/** Exit QPI mode (PM004MNIA 0xF5). */
bool rl_mram_exit_qpi(void);

#endif /* RINGLOG_BACKEND_GD32_H */



#include "ringlog_backend_gd32.h"
#include <string.h>

// ================================
// Common utilities
// ================================

static inline void delay_ms(uint32_t ms) {
    // Use GD32 HAL delay function
    delay_1ms(ms);
}

// ================================
// I2C FRAM backend
// ================================

typedef struct {
    uint32_t i2c_periph;
    uint16_t addr8;        // HAL uses 8-bit address (7-bit << 1)
    bool     addr16;       // true => 16-bit memory addresses
    uint32_t base;
    uint32_t size;
} gd32_i2c_fram_ctx_t;

static gd32_i2c_fram_ctx_t g_i2c_fram = {0};

static HAL_StatusTypeDef gd32_i2c_fram_init(void) {
    // FRAM typically has negligible power-up latency
    return HAL_OK;
}

static HAL_StatusTypeDef gd32_i2c_fram_read(uint32_t addr, uint8_t* buf, uint16_t len) {
    // Use GD32 I2C memory read (adjust to your HAL/LL)
    if (g_i2c_fram.addr16) {
        return HAL_I2C_Mem_Read((I2C_HandleTypeDef*)g_i2c_fram.i2c_periph,
                                g_i2c_fram.addr8, (uint16_t)addr,
                                I2C_MEMADD_SIZE_16BIT, buf, len, HAL_MAX_DELAY);
    } else {
        return HAL_I2C_Mem_Read((I2C_HandleTypeDef*)g_i2c_fram.i2c_periph,
                                g_i2c_fram.addr8, (uint8_t)addr,
                                I2C_MEMADD_SIZE_8BIT,  buf, len, HAL_MAX_DELAY);
    }
}

static HAL_StatusTypeDef gd32_i2c_fram_write(uint32_t addr, const uint8_t* buf, uint16_t len) {
    // FRAM supports byte writes; page size limits vary by device
    if (g_i2c_fram.addr16) {
        return HAL_I2C_Mem_Write((I2C_HandleTypeDef*)g_i2c_fram.i2c_periph,
                                 g_i2c_fram.addr8, (uint16_t)addr,
                                 I2C_MEMADD_SIZE_16BIT, (uint8_t*)buf, len, HAL_MAX_DELAY);
    } else {
        return HAL_I2C_Mem_Write((I2C_HandleTypeDef*)g_i2c_fram.i2c_periph,
                                 g_i2c_fram.addr8, (uint8_t)addr,
                                 I2C_MEMADD_SIZE_8BIT,  (uint8_t*)buf, len, HAL_MAX_DELAY);
    }
}

// Exported storage instance
ringlog_storage_t RING_GD32_I2C_FRAM = {
    .init            = gd32_i2c_fram_init,
    .read            = gd32_i2c_fram_read,
    .write           = gd32_i2c_fram_write,
    .base            = 0,
    .size            = 0,      // set by binder
    .min_write_burst = 1
};

void rl_bind_gd32_i2c_fram(uint32_t i2c_periph, uint16_t fram_addr8,
                           uint32_t base, uint32_t size, bool addr16)
{
    g_i2c_fram.i2c_periph = i2c_periph;
    g_i2c_fram.addr8      = fram_addr8;
    g_i2c_fram.addr16     = addr16;
    g_i2c_fram.base       = base;
    g_i2c_fram.size       = size;

    RING_GD32_I2C_FRAM.base = base;
    RING_GD32_I2C_FRAM.size = size;
}

// ================================
// SPI FRAM backend
// ================================

typedef struct {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef*      cs_gpio;
    uint16_t           cs_pin;
    uint32_t           base;
    uint32_t           size;
    bool               requires_wren; // many SPI FRAMs require WREN before write
} gd32_spi_fram_ctx_t;

static gd32_spi_fram_ctx_t g_spi_fram = {0};

static inline void FRAM_CS_LOW(void)  { HAL_GPIO_WritePin(g_spi_fram.cs_gpio, g_spi_fram.cs_pin, GPIO_PIN_RESET); }
static inline void FRAM_CS_HIGH(void) { HAL_GPIO_WritePin(g_spi_fram.cs_gpio, g_spi_fram.cs_pin, GPIO_PIN_SET); }

static HAL_StatusTypeDef gd32_spi_fram_init(void) {
    // FRAM often needs minimal init; add mode register writes here if required
    return HAL_OK;
}

static HAL_StatusTypeDef gd32_spi_fram_write(uint32_t addr, const uint8_t* buf, uint16_t len) {
    uint8_t wren = 0x06; // Write Enable (if required)
    uint8_t hdr[4] = {0x02, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr}; // WRITE(0x02)

    if (g_spi_fram.requires_wren) {
        FRAM_CS_LOW();
        HAL_StatusTypeDef st = HAL_SPI_Transmit(g_spi_fram.hspi, &wren, 1, HAL_MAX_DELAY);
        FRAM_CS_HIGH();
        if (st != HAL_OK) return st;
    }

    FRAM_CS_LOW();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(g_spi_fram.hspi, hdr, sizeof(hdr), HAL_MAX_DELAY);
    if (st == HAL_OK) st = HAL_SPI_Transmit(g_spi_fram.hspi, (uint8_t*)buf, len, HAL_MAX_DELAY);
    FRAM_CS_HIGH();
    return st;
}

static HAL_StatusTypeDef gd32_spi_fram_read(uint32_t addr, uint8_t* buf, uint16_t len) {
    uint8_t hdr[4] = {0x03, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr}; // READ(0x03)
    FRAM_CS_LOW();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(g_spi_fram.hspi, hdr, sizeof(hdr), HAL_MAX_DELAY);
    if (st == HAL_OK) st = HAL_SPI_Receive(g_spi_fram.hspi, buf, len, HAL_MAX_DELAY);
    FRAM_CS_HIGH();
    return st;
}

// Exported storage instance
ringlog_storage_t RING_GD32_SPI_FRAM = {
    .init            = gd32_spi_fram_init,
    .read            = gd32_spi_fram_read,
    .write           = gd32_spi_fram_write,
    .base            = 0,
    .size            = 0,      // set by binder
    .min_write_burst = 1       // FRAM supports byte writes
};

void rl_bind_gd32_spi_fram(SPI_HandleTypeDef* hspi,
                           GPIO_TypeDef* cs_gpio, uint16_t cs_pin,
                           uint32_t base, uint32_t size, bool requires_wren)
{
    g_spi_fram.hspi         = hspi;
    g_spi_fram.cs_gpio      = cs_gpio;
    g_spi_fram.cs_pin       = cs_pin;
    g_spi_fram.base         = base;
    g_spi_fram.size         = size;
    g_spi_fram.requires_wren= requires_wren;

    RING_GD32_SPI_FRAM.base = base;
    RING_GD32_SPI_FRAM.size = size;
}

// ================================
// OSPI/QSPI MRAM (PM004MNIA) backend
// ================================

typedef struct {
    uint32_t                 ospi_periph;
    ospi_parameter_struct*   ospi_cfg;
    bool                     qpi;   // true: 4-line QPI; false: SPI 1-line
    uint32_t                 base;
    uint32_t                 size;
} gd32_ospi_mram_ctx_t;

static gd32_ospi_mram_ctx_t g_mram = {0};

// Helpers: send a simple command (no address/data)
static bool mram_send_simple_cmd(uint8_t instr, uint8_t ins_lines) {
    ospi_regular_cmd_struct cmd = {0};
    cmd.operation_type      = OSPI_OPTYPE_COMMON_CFG;
    cmd.ins_mode            = ins_lines;
    cmd.instruction         = instr;
    cmd.ins_size            = OSPI_INSTRUCTION_8_BITS;
    cmd.addr_mode           = OSPI_ADDRESS_NONE;
    cmd.data_mode           = OSPI_DATA_NONE;
    cmd.dummy_cycles        = OSPI_DUMYC_CYCLES_0;
    cmd.addr_dtr_mode       = OSPI_ADDRDTR_MODE_DISABLE;
    cmd.alter_bytes_mode    = OSPI_ALTERNATE_BYTES_NONE;
    cmd.alter_bytes_dtr_mode= OSPI_ABDTR_MODE_DISABLE;
    cmd.data_dtr_mode       = OSPI_DADTR_MODE_DISABLE;
    return (ospi_command_config(g_mram.ospi_periph, g_mram.ospi_cfg, &cmd) == SUCCESS);
}

// Public QPI helpers
bool rl_mram_enter_qpi(void) {
    // Enter QPI is single-line instruction 0x35
    return mram_send_simple_cmd(0x35, OSPI_INSTRUCTION_1_LINE);
}
bool rl_mram_exit_qpi(void) {
    // Exit QPI can be accepted in QPI (4-lines) on many devices; adjust if needed
    return mram_send_simple_cmd(0xF5, OSPI_INSTRUCTION_4_LINES);
}

static HAL_StatusTypeDef gd32_ospi_mram_init(void) {
    // Datasheet: power-up delay tPU ≈ 1.5 ms
    delay_ms(2);
    // If user requested QPI, enter QPI now
    if (g_mram.qpi) {
        if (!rl_mram_enter_qpi()) return HAL_ERROR;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef gd32_ospi_mram_write(uint32_t addr, const uint8_t* buf, uint16_t len) {
    // PM004MNIA requires minimum 2-byte burst
    if (len & 0x1) return HAL_ERROR;

    // Write Enable (0x06)
    if (!mram_send_simple_cmd(0x06, g_mram.qpi ? OSPI_INSTRUCTION_4_LINES : OSPI_INSTRUCTION_1_LINE))
        return HAL_ERROR;

    // WRITE (0x02) + 24-bit address + data; no dummy
    ospi_regular_cmd_struct cmd = {0};
    cmd.operation_type      = OSPI_OPTYPE_COMMON_CFG;
    cmd.ins_mode            = g_mram.qpi ? OSPI_INSTRUCTION_4_LINES : OSPI_INSTRUCTION_1_LINE;
    cmd.instruction         = 0x02;
    cmd.ins_size            = OSPI_INSTRUCTION_8_BITS;

    cmd.addr_mode           = g_mram.qpi ? OSPI_ADDRESS_4_LINES    : OSPI_ADDRESS_1_LINE;
    cmd.addr_size           = OSPI_ADDRESS_24_BITS;
    cmd.address             = addr;
    cmd.addr_dtr_mode       = OSPI_ADDRDTR_MODE_DISABLE;

    cmd.alter_bytes_mode    = OSPI_ALTERNATE_BYTES_NONE;
    cmd.data_mode           = g_mram.qpi ? OSPI_DATA_4_LINES       : OSPI_DATA_1_LINE;
    cmd.data_dtr_mode       = OSPI_DADTR_MODE_DISABLE;
    cmd.dummy_cycles        = OSPI_DUMYC_CYCLES_0;
    cmd.nbdata              = len;

    if (ospi_command_config(g_mram.ospi_periph, g_mram.ospi_cfg, &cmd) != SUCCESS)
        return HAL_ERROR;
    if (ospi_transmit(g_mram.ospi_periph, (uint8_t*)buf) != SUCCESS)
        return HAL_ERROR;

    return HAL_OK;
}

static HAL_StatusTypeDef gd32_ospi_mram_read(uint32_t addr, uint8_t* buf, uint16_t len) {
    // READ (0x03) + 24-bit address; default no dummy (MR#2 LT=0). Set dummy cycles if you configured LT>0.
    ospi_regular_cmd_struct cmd = {0};
    cmd.operation_type      = OSPI_OPTYPE_COMMON_CFG;
    cmd.ins_mode            = g_mram.qpi ? OSPI_INSTRUCTION_4_LINES : OSPI_INSTRUCTION_1_LINE;
    cmd.instruction         = 0x03;
    cmd.ins_size            = OSPI_INSTRUCTION_8_BITS;

    cmd.addr_mode           = g_mram.qpi ? OSPI_ADDRESS_4_LINES    : OSPI_ADDRESS_1_LINE;
    cmd.addr_size           = OSPI_ADDRESS_24_BITS;
    cmd.address             = addr;
    cmd.addr_dtr_mode       = OSPI_ADDRDTR_MODE_DISABLE;

    cmd.alter_bytes_mode    = OSPI_ALTERNATE_BYTES_NONE;
    cmd.data_mode           = g_mram.qpi ? OSPI_DATA_4_LINES       : OSPI_DATA_1_LINE;
    cmd.data_dtr_mode       = OSPI_DADTR_MODE_DISABLE;
    cmd.dummy_cycles        = OSPI_DUMYC_CYCLES_0;   // change if MR#2 LT != 0
    cmd.nbdata              = len;

    if (ospi_command_config(g_mram.ospi_periph, g_mram.ospi_cfg, &cmd) != SUCCESS)
        return HAL_ERROR;
    if (ospi_receive(g_mram.ospi_periph, buf) != SUCCESS)
        return HAL_ERROR;

    return HAL_OK;
}

// Exported storage instance
ringlog_storage_t RING_GD32_OSPI_MRAM_PM004 = {
    .init            = gd32_ospi_mram_init,
    .read            = gd32_ospi_mram_read,
    .write           = gd32_ospi_mram_write,
    .base            = 0,
    .size            = 0,      // set by binder
    .min_write_burst = 2       // PM004MNIA requires min 2-byte burst
};

void rl_bind_gd32_ospi_mram_pm004(uint32_t ospi_periph, ospi_parameter_struct* ospi_cfg,
                                  uint32_t base, uint32_t size, bool use_qpi)
{
    g_mram.ospi_periph = ospi_periph;
    g_mram.ospi_cfg    = ospi_cfg;
    g_mram.qpi         = use_qpi;
    g_mram.base        = base;
    g_mram.size        = size;

    RING_GD32_OSPI_MRAM_PM004.base = base;
    RING_GD32_OSPI_MRAM_PM004.size = size;
}

#include "ringlog_backend_gd32.h"
#include "ringlog.h"

void app_init(void)
{
    // 1) Choose a backend and bind its hardware context
    // Example: PM004MNIA MRAM over OSPI, QPI enabled
    rl_bind_gd32_ospi_mram_pm004(OSPI0, &ospi_cfg0, 0, 512*1024, true);

    // 2) Hand the storage to ringlog
    ringlog_bind_storage(&RING_GD32_OSPI_MRAM_PM004);

    // 3) Cold boot recover → locates head via binary search
    if (!ringlog_cold_boot_recover()) {
        // Handle error / first boot
    }
}

void app_write_16B(const uint8_t payload[16])
{
    // Append one 16-byte record (two-phase commit handled in ringlog.c)
    HAL_StatusTypeDef st = ringlog_append(payload);
    // Check st for OK / error
}

uint32_t mram_read_unique_id_spi(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct)
{
    uint8_t uid[16];
    ospi_regular_cmd_struct cmd = {0};

    // PM004MNIA SPI Unique ID: 0x9F + 0x00 0x00 0x00, read 16 bytes, no dummy
    cmd.operation_type      = OSPI_OPTYPE_COMMON_CFG;
    cmd.ins_mode            = OSPI_INSTRUCTION_1_LINE;
    cmd.instruction         = 0x9F;                    // Read Unique ID
    cmd.ins_size            = OSPI_INSTRUCTION_8_BITS;

    cmd.addr_mode           = OSPI_ADDRESS_1_LINE;     // MUST send 3 zero bytes
    cmd.addr_size           = OSPI_ADDRESS_24_BITS;
    cmd.address             = 0x000000;                // three 0x00 bytes
    cmd.addr_dtr_mode       = OSPI_ADDRDTR_MODE_DISABLE;

    cmd.alter_bytes_mode    = OSPI_ALTERNATE_BYTES_NONE;
    cmd.data_mode           = OSPI_DATA_1_LINE;
    cmd.data_dtr_mode       = OSPI_DADTR_MODE_DISABLE;
    cmd.dummy_cycles        = OSPI_DUMYC_CYCLES_0;
    cmd.nbdata              = 16;                      // MUST read 16 bytes

    // send command
    ospi_command_config(ospi_periph, ospi_struct, &cmd);

    // receive data (16B UID: first two bytes are Manufacturer ID)
    ospi_receive(ospi_periph, uid);

    // pack first 4 bytes as a convenience (you may need all 16 bytes)
    return ( (uint32_t)uid[0] << 24 ) | ( (uint32_t)uid[1] << 16 ) |
           ( (uint32_t)uid[2] << 8  ) | ( (uint32_t)uid[3] );
}

void mram_enter_qpi(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct)
{
    ospi_regular_cmd_struct cmd = {0};
    cmd.operation_type   = OSPI_OPTYPE_COMMON_CFG;
    cmd.ins_mode         = OSPI_INSTRUCTION_1_LINE;      // Enter QPI is single-line instruction
    cmd.instruction      = 0x35;                         // Enter QPI
    cmd.ins_size         = OSPI_INSTRUCTION_8_BITS;
    cmd.addr_mode        = OSPI_ADDRESS_NONE;
    cmd.data_mode        = OSPI_DATA_NONE;
    cmd.dummy_cycles     = OSPI_DUMYC_CYCLES_0;
    ospi_command_config(ospi_periph, ospi_struct, &cmd);
}

void mram_exit_qpi(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct)
{
    ospi_regular_cmd_struct cmd = {0};
    cmd.operation_type   = OSPI_OPTYPE_COMMON_CFG;
    cmd.ins_mode         = OSPI_INSTRUCTION_4_LINES;      // or 1-line depending on device state
    cmd.instruction      = 0xF5;                         // Exit QPI
    cmd.ins_size         = OSPI_INSTRUCTION_8_BITS;
    cmd.addr_mode        = OSPI_ADDRESS_NONE;
    cmd.data_mode        = OSPI_DATA_NONE;
    cmd.dummy_cycles     = OSPI_DUMYC_CYCLES_0;
    ospi_command_config(ospi_periph, ospi_struct, &cmd);
}

uint32_t mram_read_unique_id_qpi(uint32_t ospi_periph, ospi_parameter_struct *ospi_struct)
{
    uint8_t uid[16];
    ospi_regular_cmd_struct cmd = {0};

    // In QPI, instruction/address/data use parallel lines; no dummy; 16B data
    cmd.operation_type      = OSPI_OPTYPE_COMMON_CFG;
    cmd.ins_mode            = OSPI_INSTRUCTION_4_LINES;   // Quad instruction width (if GD32 API supports 4 lines)
    cmd.instruction         = 0x9F;
    cmd.ins_size            = OSPI_INSTRUCTION_8_BITS;

    cmd.addr_mode           = OSPI_ADDRESS_4_LINES;
    cmd.addr_size           = OSPI_ADDRESS_24_BITS;
    cmd.address             = 0x000000;
    cmd.addr_dtr_mode       = OSPI_ADDRDTR_MODE_DISABLE;

    cmd.alter_bytes_mode    = OSPI_ALTERNATE_BYTES_NONE;
    cmd.data_mode           = OSPI_DATA_4_LINES;
    cmd.data_dtr_mode       = OSPI_DADTR_MODE_DISABLE;
    cmd.dummy_cycles        = OSPI_DUMYC_CYCLES_0;
    cmd.nbdata              = 16;

    ospi_command_config(ospi_periph, ospi_struct, &cmd);
    ospi_receive(ospi_periph, uid);

    return ( (uint32_t)uid[0] << 24 ) | ( (uint32_t)uid[1] << 16 ) |
           ( (uint32_t)uid[2] << 8  ) | ( (uint32_t)uid[3] );
}

// mram_uid.h
#ifndef MRAM_UID_H
#define MRAM_UID_H

#include <stdint.h>
#include <stdbool.h>
#include "gd32h7xx.h"                 // 根据你的工程路径包含GD32H759外设头
#include "gd32h7xx_ospi.h"            // OSPI寄存器/结构体定义（名称按你的SDK调整）

#ifdef __cplusplus
extern "C" {
#endif

// ---- MRAM Commands (PM004MNIA) ----
#define MRAM_CMD_READ_UID      0x9F    // Read Unique ID
#define MRAM_CMD_ENTER_QPI     0x35    // Enter QPI
#define MRAM_CMD_EXIT_QPI      0xF5    // Exit QPI

// ---- UID length ----
#define MRAM_UID_LEN           16U

// ---- Optional: power-up delay in ms (datasheet tPU ≈ 1.5 ms) ----
#define MRAM_POWER_UP_DELAY_MS 2U

// Interface selector (your project may already define a similar enum)
typedef enum {
    MRAM_BUS_SPI_1LINE = 0,    // SPI 1-line (always supported)
    MRAM_BUS_QPI_4LINES        // Quad I/O (requires device entered QPI and OSPI support)
} mram_bus_mode_t;

// UID parsed view (you can extend fields as needed)
typedef struct {
    uint8_t raw[MRAM_UID_LEN];  // full 16 bytes
    uint8_t mfg_id0;            // raw[0]
    uint8_t mfg_id1;            // raw[1]
    uint8_t device_id0;         // raw[2]
    uint8_t device_id1;         // raw[3]
} mram_uid_t;

// API:
// 1) Read UID in SPI 1-line (no QPI enter/exit required)
// 2) Read UID in QPI 4-lines (will NOT send enter/exit here; call helpers if needed)
// 3) Helpers to enter/exit QPI (optional)
// 4) A convenience wrapper that does SPI-only read (recommended unless you already use QPI)

bool mram_uid_read_spi(uint32_t ospi_periph, ospi_parameter_struct* ospi_struct, mram_uid_t* out);
bool mram_uid_read_qpi(uint32_t ospi_periph, ospi_parameter_struct* ospi_struct, mram_uid_t* out);

bool mram_enter_qpi(uint32_t ospi_periph, ospi_parameter_struct* ospi_struct);
bool mram_exit_qpi (uint32_t ospi_periph, ospi_parameter_struct* ospi_struct);

// Convenience wrapper: choose by mode; if mode unsupported, fallback to SPI
bool mram_uid_read(uint32_t ospi_periph, ospi_parameter_struct* ospi_struct,
                   mram_bus_mode_t mode, mram_uid_t* out);

#ifdef __cplusplus
}
#endif
#endif // MRAM_UID_H

// mram_uid.c
#include "mram_uid.h"
#include <string.h>

// ---- Local helpers ----
static inline void mram_powerup_delay(void) {
    // PM004MNIA datasheet: tPU ≈ 1.5 ms; use 2 ms for safety
    delay_1ms(MRAM_POWER_UP_DELAY_MS);
}

static bool mram_send_simple_cmd(uint32_t ospi_periph,
                                 ospi_parameter_struct* ospi_struct,
                                 uint8_t instr,
                                 uint8_t ins_lines)
{
    ospi_regular_cmd_struct cmd = {0};
    cmd.operation_type      = OSPI_OPTYPE_COMMON_CFG;
    cmd.instruction         = instr;
    cmd.ins_size            = OSPI_INSTRUCTION_8_BITS;
    cmd.addr_mode           = OSPI_ADDRESS_NONE;
    cmd.data_mode           = OSPI_DATA_NONE;
    cmd.dummy_cycles        = OSPI_DUMYC_CYCLES_0;
    cmd.addr_dtr_mode       = OSPI_ADDRDTR_MODE_DISABLE;
    cmd.alter_bytes_mode    = OSPI_ALTERNATE_BYTES_NONE;
    cmd.alter_bytes_dtr_mode= OSPI_ABDTR_MODE_DISABLE;
    cmd.data_dtr_mode       = OSPI_DADTR_MODE_DISABLE;

    // instruction line mode
    cmd.ins_mode = ins_lines;

    // send command
    if (ospi_command_config(ospi_periph, ospi_struct, &cmd) != SUCCESS) {
        return false;
    }
    return true;
}

// ---- Public: Enter / Exit QPI ----

bool mram_enter_qpi(uint32_t ospi_periph, ospi_parameter_struct* ospi_struct)
{
    // Enter QPI typically uses single-line instruction 0x35
    return mram_send_simple_cmd(ospi_periph, ospi_struct,
                                MRAM_CMD_ENTER_QPI, OSPI_INSTRUCTION_1_LINE);
}

bool mram_exit_qpi(uint32_t ospi_periph, ospi_parameter_struct* ospi_struct)
{
    // Exit QPI can be accepted in QPI (4-lines) on some devices; if not, try 1-line
    // Here we use 4-lines first; adjust if your part requires 1-line exit.
    return mram_send_simple_cmd(ospi_periph, ospi_struct,
                                MRAM_CMD_EXIT_QPI, OSPI_INSTRUCTION_4_LINES);
}

// ---- Public: Read UID in SPI 1-line ----

bool mram_uid_read_spi(uint32_t ospi_periph, ospi_parameter_struct* ospi_struct, mram_uid_t* out)
{
    if (!out) return false;
    memset(out, 0, sizeof(*out));

    // Ensure power-up delay; skip if already ensured earlier in your init sequence.
    mram_powerup_delay();

    uint8_t uid[MRAM_UID_LEN];

    ospi_regular_cmd_struct cmd = {0};
    cmd.operation_type      = OSPI_OPTYPE_COMMON_CFG;

    // Instruction phase (SPI 1-line)
    cmd.ins_mode            = OSPI_INSTRUCTION_1_LINE;
    cmd.instruction         = MRAM_CMD_READ_UID;         // 0x9F
    cmd.ins_size            = OSPI_INSTRUCTION_8_BITS;

    // Address phase: MUST send 3 bytes 0x00 0x00 0x00
    cmd.addr_mode           = OSPI_ADDRESS_1_LINE;
    cmd.addr_size           = OSPI_ADDRESS_24_BITS;
    cmd.address             = 0x000000;
    cmd.addr_dtr_mode       = OSPI_ADDRDTR_MODE_DISABLE;

    // No alternate bytes
    cmd.alter_bytes_mode    = OSPI_ALTERNATE_BYTES_NONE;
    cmd.alter_bytes_dtr_mode= OSPI_ABDTR_MODE_DISABLE;

    // Data phase: 1-line, 16 bytes, no dummy
    cmd.data_mode           = OSPI_DATA_1_LINE;
    cmd.data_dtr_mode       = OSPI_DADTR_MODE_DISABLE;
    cmd.dummy_cycles        = OSPI_DUMYC_CYCLES_0;
    cmd.nbdata              = MRAM_UID_LEN;

    if (ospi_command_config(ospi_periph, ospi_struct, &cmd) != SUCCESS) {
        return false;
    }
    if (ospi_receive(ospi_periph, uid) != SUCCESS) {
        return false;
    }

    // Fill result
    memcpy(out->raw, uid, MRAM_UID_LEN);
    out->mfg_id0   = uid[0];
    out->mfg_id1   = uid[1];
    out->device_id0= uid[2];
    out->device_id1= uid[3];
    return true;
}

// ---- Public: Read UID in QPI 4-lines (device must be in QPI state!) ----

bool mram_uid_read_qpi(uint32_t ospi_periph, ospi_parameter_struct* ospi_struct, mram_uid_t* out)
{
    if (!out) return false;
    memset(out, 0, sizeof(*out));

    // Assume device already in QPI (via 0x35); no extra delay needed here.
    uint8_t uid[MRAM_UID_LEN];

    ospi_regular_cmd_struct cmd = {0};
    cmd.operation_type      = OSPI_OPTYPE_COMMON_CFG;

    // In QPI: instruction/address/data are 4 lines (if your GD32 OSPI supports 4-lines config)
    cmd.ins_mode            = OSPI_INSTRUCTION_4_LINES;
    cmd.instruction         = MRAM_CMD_READ_UID;         // 0x9F
    cmd.ins_size            = OSPI_INSTRUCTION_8_BITS;

    cmd.addr_mode           = OSPI_ADDRESS_4_LINES;
    cmd.addr_size           = OSPI_ADDRESS_24_BITS;
    cmd.address             = 0x000000;
    cmd.addr_dtr_mode       = OSPI_ADDRDTR_MODE_DISABLE;

    cmd.alter_bytes_mode    = OSPI_ALTERNATE_BYTES_NONE;
    cmd.alter_bytes_dtr_mode= OSPI_ABDTR_MODE_DISABLE;

    cmd.data_mode           = OSPI_DATA_4_LINES;
    cmd.data_dtr_mode       = OSPI_DADTR_MODE_DISABLE;
    cmd.dummy_cycles        = OSPI_DUMYC_CYCLES_0;
    cmd.nbdata              = MRAM_UID_LEN;

    if (ospi_command_config(ospi_periph, ospi_struct, &cmd) != SUCCESS) {
        return false;
    }
    if (ospi_receive(ospi_periph, uid) != SUCCESS) {
        return false;
    }

    memcpy(out->raw, uid, MRAM_UID_LEN);
    out->mfg_id0   = uid[0];
    out->mfg_id1   = uid[1];
    out->device_id0= uid[2];
    out->device_id1= uid[3];
    return true;
}

// ---- Public: Convenience wrapper ----

bool mram_uid_read(uint32_t ospi_periph, ospi_parameter_struct* ospi_struct,
                   mram_bus_mode_t mode, mram_uid_t* out)
{
    if (mode == MRAM_BUS_QPI_4LINES) {
        // If your system already runs the MRAM in QPI, just do QPI read:
        // (Optionally, call mram_enter_qpi() before and mram_exit_qpi() after)
        return mram_uid_read_qpi(ospi_periph, ospi_struct, out);
    }
    // Default: SPI 1-line
    return mram_uid_read_spi(ospi_periph, ospi_struct, out);
}

#include "mram_uid.h"

// 假设你已完成 OSPI 外设/时钟/管脚初始化，并持有 ospi_periph 与 ospi_struct
void demo_read_mram_uid(void)
{
    mram_uid_t uid;
    bool ok;

    // 方案A：SPI（推荐，最通用）
    ok = mram_uid_read(OSPI0, &ospi_struct0, MRAM_BUS_SPI_1LINE, &uid);
    if (ok) {
        // 常见返回：uid.raw[0..1] 为 Manufacturer ID（PM004MNIA文档举例0x29, 0x55）
        // 其余字节可按需要解析/打印
    }

    // 方案B：QPI（仅当你确定设备和库支持4线且已经进入QPI）
    /*
    if (mram_enter_qpi(OSPI0, &ospi_struct0)) {
        ok = mram_uid_read(OSPI0, &ospi_struct0, MRAM_BUS_QPI_4LINES, &uid);
        mram_exit_qpi(OSPI0, &ospi_struct0);
    }
    */
}
