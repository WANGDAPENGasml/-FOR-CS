
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* 公开的 MDMA 接口 */
HAL_StatusTypeDef LCD_MDMA_Init(void);
HAL_StatusTypeDef NT35510_Blit_MDMA(uint16_t x, uint16_t y,
                                    uint16_t w, uint16_t h,
                                    const uint16_t *src_rgb565);

HAL_StatusTypeDef NT35510_FillRect_MDMA(uint16_t x, uint16_t y,
                                        uint16_t w, uint16_t h,
                                        uint16_t color);

/* 可选：查询传输忙闲 */
uint8_t LCD_MDMA_IsBusy(void);

#ifdef __cplusplus
}
#endif

/* =====================  MDMA 加速块传输实现  ===================== */
#include "string.h"

#ifndef ARRAY_LEN
#define ARRAY_LEN(a) (sizeof(a)/sizeof((a)[0]))
#endif

/* LCD 数据寄存器的固定写入地址（16bit） */
#define LCD_GRAM_ADDR   ((uint32_t)&*(volatile uint16_t*)(FMC_Addr_NT35510_DATA))

/* MDMA 句柄与忙标志 */
static MDMA_HandleTypeDef hmdma_lcd;
static volatile uint8_t s_mdma_busy = 0;

/* 对齐到 32-byte cache line，清缓存辅助 */
static void DCache_CleanByAddr_Aligned(const void *addr, size_t len)
{
#if (__DCACHE_PRESENT == 1U)
    uintptr_t a   = (uintptr_t)addr;
    uintptr_t a0  = a & ~(uintptr_t)31U;
    size_t    len_aligned = (a + len - a0 + 31U) & ~(size_t)31U;
    SCB_CleanDCache_by_Addr((uint32_t*)a0, len_aligned);
#else
    (void)addr; (void)len;
#endif
}

/* MDMA 传输完成/错误回调 */
void HAL_MDMA_XferCpltCallback(MDMA_HandleTypeDef *h)
{
    if (h == &hmdma_lcd) s_mdma_busy = 0;
}
void HAL_MDMA_XferErrorCallback(MDMA_HandleTypeDef *h)
{
    if (h == &hmdma_lcd) s_mdma_busy = 0;
}

/* LCD MDMA 初始化：软件触发、块传输、源/目的 16bit，目的不递增 */
HAL_StatusTypeDef LCD_MDMA_Init(void)
{
    __HAL_RCC_MDMA_CLK_ENABLE();

    hmdma_lcd.Instance = MDMA_Channel0;                /* 任意空闲通道 */
    hmdma_lcd.Init.Request              = MDMA_REQUEST_SW;             /* 软件触发 */
    hmdma_lcd.Init.TransferTriggerMode  = MDMA_BLOCK_TRANSFER;         /* 块传输 */
    hmdma_lcd.Init.Priority             = MDMA_PRIORITY_HIGH;
    hmdma_lcd.Init.Endianness           = MDMA_LITTLE_ENDIANNESS_PRESERVE;

    /* 源递增：半字；目的不递增：半字（写同一 GRAM 地址） */
    hmdma_lcd.Init.SourceInc            = MDMA_SRC_INC_HALFWORD;
    hmdma_lcd.Init.DestinationInc       = MDMA_DEST_INC_DISABLE;
    hmdma_lcd.Init.SourceDataSize       = MDMA_SRC_DATASIZE_HALFWORD;
    hmdma_lcd.Init.DestDataSize         = MDMA_DEST_DATASIZE_HALFWORD;

    /* 数据对齐/突发参数 */
    hmdma_lcd.Init.DataAlignment        = MDMA_DATAALIGN_PACKENABLE;
    hmdma_lcd.Init.BufferTransferLength = 128;     /* 每触发搬 128*2 字节，可按需调优 */
    hmdma_lcd.Init.SourceBurst          = MDMA_SOURCE_BURST_32BEATS;
    hmdma_lcd.Init.DestBurst            = MDMA_DEST_BURST_32BEATS;

    /* Block 偏移：常规 0 */
    hmdma_lcd.Init.SourceBlockAddressOffset      = 0;
    hmdma_lcd.Init.DestBlockAddressOffset        = 0;

    if (HAL_MDMA_Init(&hmdma_lcd) != HAL_OK) return HAL_ERROR;

    /* 中断配置（可根据你的中断向量放到 MSP 中） */
    HAL_NVIC_SetPriority(MDMA_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(MDMA_IRQn);

    s_mdma_busy = 0;
    return HAL_OK;
}

/* 在中断文件 stm32h7xx_it.c 中添加：
void MDMA_IRQHandler(void) { HAL_MDMA_IRQHandler(&hmdma_lcd); }
*/

/* 内部：分段启动一次 MDMA 传输（长度单位：半字） */
static HAL_StatusTypeDef LCD_MDMA_Xfer(const uint16_t *src, uint32_t halfwords)
{
    if (halfwords == 0) return HAL_OK;

    s_mdma_busy = 1;

    /* Cache 清理，确保 SRAM 中的像素已回写到内存 */
    DCache_CleanByAddr_Aligned(src, halfwords * sizeof(uint16_t));

    /* HAL_MDMA_Start_IT 的参数：SrcAddr, DstAddr, BlockDataLength, BlockCount
       这里选 1 个块，块长度=halfwords（半字个数） */
    if (HAL_MDMA_Start_IT(&hmdma_lcd,
                          (uint32_t)src,
                          LCD_GRAM_ADDR,
                          halfwords,           /* 每块传输元素数（半字） */
                          1                    /* 块次数 */
                         ) != HAL_OK)
    {
        s_mdma_busy = 0;
        return HAL_ERROR;
    }

    /* 轮询或等待回调，这里简化为轮询 */
    if (HAL_MDMA_PollForTransfer(&hmdma_lcd, HAL_MDMA_FULL_TRANSFER, 0xFFFFFF) != HAL_OK)
    {
        s_mdma_busy = 0;
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* 对外：MDMA 区域贴图（RGB565），自动切分 64K-1 大小段 */
HAL_StatusTypeDef NT35510_Blit_MDMA(uint16_t x, uint16_t y,
                                    uint16_t w, uint16_t h,
                                    const uint16_t *src_rgb565)
{
    if ((w == 0) || (h == 0) || (src_rgb565 == NULL)) return HAL_ERROR;

    /* 设置窗口并进入 Memory Write 状态（用你已有 API） */
    NT35510_OpenWindow(x, y, w, h);
    NT35510_Write_Cmd(CMD_SetPixel); /* 等效于 0x2C */

    /* 连续写入 w*h 个半字到 LCD_GRAM_ADDR */
    uint32_t remain  = (uint32_t)w * (uint32_t)h;
    const uint16_t *p = src_rgb565;

    /* HAL_MDMA 的块长度通常 16bits 计数最大 65535，做分段更稳妥 */
    while (remain)
    {
        uint32_t this_len = (remain > 65535U) ? 65535U : remain;
        HAL_StatusTypeDef st = LCD_MDMA_Xfer(p, this_len);
        if (st != HAL_OK) return st;
        p      += this_len;
        remain -= this_len;
    }
    return HAL_OK;
}

/* 对外：MDMA 纯色填充（源固定地址复用同一半字），同样切段执行 */
HAL_StatusTypeDef NT35510_FillRect_MDMA(uint16_t x, uint16_t y,
                                        uint16_t w, uint16_t h,
                                        uint16_t color)
{
    if ((w == 0) || (h == 0)) return HAL_ERROR;

    NT35510_OpenWindow(x, y, w, h);
    NT35510_Write_Cmd(CMD_SetPixel); /* 0x2C */

    /* 临时切换为“源不递增”，实现单值复制 */
    uint32_t backup_src_inc = hmdma_lcd.Init.SourceInc;
    hmdma_lcd.Init.SourceInc = MDMA_SRC_INC_DISABLE;
    HAL_MDMA_DeInit(&hmdma_lcd);
    if (HAL_MDMA_Init(&hmdma_lcd) != HAL_OK) {
        /* 失败恢复 */
        hmdma_lcd.Init.SourceInc = backup_src_inc;
        HAL_MDMA_Init(&hmdma_lcd);
        return HAL_ERROR;
    }

    uint32_t remain = (uint32_t)w * (uint32_t)h;
    while (remain)
    {
        uint32_t this_len = (remain > 65535U) ? 65535U : remain;

        s_mdma_busy = 1;
        /* 源/目的都固定地址：从同一半字不断写到 LCD_GRAM_ADDR */
        if (HAL_MDMA_Start_IT(&hmdma_lcd,
                              (uint32_t)&color,
                              LCD_GRAM_ADDR,
                              this_len, 1) != HAL_OK)
        {
            s_mdma_busy = 0;
            break;
        }
        if (HAL_MDMA_PollForTransfer(&hmdma_lcd, HAL_MDMA_FULL_TRANSFER, 0xFFFFFF) != HAL_OK)
        {
            s_mdma_busy = 0;
            break;
        }
        remain -= this_len;
    }

    /* 恢复“源递增”配置 */
    hmdma_lcd.Init.SourceInc = backup_src_inc;
    HAL_MDMA_DeInit(&hmdma_lcd);
    HAL_MDMA_Init(&hmdma_lcd);

    return (remain == 0) ? HAL_OK : HAL_ERROR;
}

uint8_t LCD_MDMA_IsBusy(void) { return s_mdma_busy; }

extern MDMA_HandleTypeDef hmdma_lcd;
void MDMA_IRQHandler(void)
{
    HAL_MDMA_IRQHandler(&hmdma_lcd);
}



/* 1) 初始化顺序：FMC/NT35510 正常初始化后，再初始化 MDMA */
NT35510_Init();           /* 你现有的初始化，含 FMC/寄存器/清屏/背光等 */
LCD_MDMA_Init();          /* 新增：开启 MDMA */

/* 2) 纯色填充（DMA 版） */
NT35510_FillRect_MDMA(0, 0, LCD_X_LENGTH, LCD_Y_LENGTH, 0x0000);  // 全屏黑

/* 3) 区域贴图（RGB565） */
static uint16_t linebuf[800];   // 举例：800 像素一行的条带（AXI SRAM）
prepare_line(linebuf, 800);
NT35510_Blit_MDMA(0, 100, 800, 1, linebuf);   // 在 y=100 位置刷一条 800x1


// 上电初始化
NT35510_Init();          // 你已有：含复位、寄存器序列、FMC 时序、清屏、背光等
LCD_MDMA_Init();         // 新增：开启 MDMA

// 竖屏基准：X=480, Y=800
const uint16_t LCD_W = 480;
const uint16_t LCD_H = 800;
const uint16_t BLUE   = 0x001F;

// 纯色全屏（MDMA）
NT35510_FillRect_MDMA(0, 0, LCD_W, LCD_H, BLUE);


/*
 * lcd_demo_sin2t.c  ——  NT35510 (GRAM) + FMC(8080) + MDMA 的 sin²(t) Demo
 *
 * 依赖：
 *   - 你的 bsp_nt35510_lcd.h / .c（含 NT35510_OpenWindow / CMD_SetPixel 等）
 *   - 我提供的 MDMA 版接口：LCD_MDMA_Init / NT35510_Blit_MDMA / NT35510_FillRect_MDMA
 *   - 颜色宏、LCD_X_LENGTH/LCD_Y_LENGTH 来自你的头文件
 */

#include "bsp_nt35510_lcd.h"
#include <math.h>
#include <string.h>

/* ================= 可调参数 ================= */
#ifndef LCD_W
#define LCD_W   (800)   // 横屏宽；竖屏请改为 480
#endif
#ifndef LCD_H
#define LCD_H   (480)   // 横屏高；竖屏请改为 800
#endif

/* 背景色与曲线色（RGB565）—— 使用你头文件中已有的颜色宏 */
#define COLOR_BG    BLACK
#define COLOR_AXIS  GREY
#define COLOR_CURVE YELLOW

/* tile：这里使用“1 行”做演示；如需更快，可改为多行条带，比如 8 或 16 行 */
#define TILE_LINES  1

/* 正弦参数 */
typedef struct {
    float amplitude;   // 振幅（像素）
    float baseline;    // 基线（像素）
    float phase;       // 相位 φ（弧度）
    float omega;       // 角频率 ω（弧度/像素）
} sin2t_param_t;

/* ========= 内部工具：画坐标轴（可选） ========= */
static void draw_axes(void)
{
    /* X 轴：屏幕中线；Y 轴：左侧边距 40 像素 */
    const int y_axis = LCD_H/2;
    const int x_axis = 40;

    /* 横线 */
    NT35510_FillRect_MDMA(0, y_axis, LCD_W, 1, COLOR_AXIS);
    /* 竖线 */
    NT35510_FillRect_MDMA(x_axis, 0, 1, LCD_H, COLOR_AXIS);
}

/* ========= 静态图：一帧 sin²(x) =========
 * y = baseline - amplitude * sin^2(omega * x + phase)
 * 采用“每行 tile”生成整行背景+曲线像素，然后用 MDMA 整行推送
 */
void Demo_Sin2t_Static(void)
{
    /* 1) 背景清屏 */
    NT35510_FillRect_MDMA(0, 0, LCD_W, LCD_H, COLOR_BG);

    /* 2) 可选：画坐标轴 */
    draw_axes();

    /* 3) 正弦参数（以横屏为例）*/
    sin2t_param_t p;
    p.amplitude = (float)LCD_H * 0.4f;        // 占高度 40% 的振幅
    p.baseline  = (float)LCD_H * 0.5f;        // 垂直居中
    p.phase     = 0.0f;                       // 静态：相位为 0
    p.omega     = 2.0f * (float)M_PI / 200.0f;/* 200 像素一周期 */

    /* 4) 行缓冲（tile buffer），放 AXI SRAM。每像素 16bit（RGB565） */
    static uint16_t line[LCD_W];

    for (int y = 0; y < LCD_H; y += TILE_LINES)
    {
        /* 先默认整行背景色 */
        for (int x = 0; x < LCD_W; ++x) line[x] = COLOR_BG;

        /* 计算曲线 y(x)：找到该 y 行应该点亮的 x 位置（可用最近邻或加粗 3px） */
        for (int x = 0; x < LCD_W; ++x)
        {
            float s  = sinf(p.omega * (float)x + p.phase);
            float yy = p.baseline - p.amplitude * (s*s);    // sin²
            int   yi = (int)(yy + 0.5f);

            if (yi >= 0 && yi < LCD_H && yi == y) {
                /* 在当前行命中，点亮该像素（可加粗） */
                line[x] = COLOR_CURVE;
            }
            /* 加粗 3px（垂直方向）可选：
               if (abs(yi - y) <= 1) line[x] = COLOR_CURVE;
            */
        }

        /* 5) 用 MDMA 把这一行推到 LCD（OpenWindow+0x2C 已在 NT35510_Blit_MDMA 内部处理） */
        NT35510_Blit_MDMA(0, y, LCD_W, TILE_LINES, line);
    }
}

/* ========= 动画：相位随时间推进（φ += dphi），形成流动效果 =========
 * 你可在主循环或定时器中调用；若用 RTOS，可放到一个任务里。
 */
void Demo_Sin2t_Anim(float dphi_per_frame, uint32_t frame_count)
{
    /* 1) 背景清屏一次 */
    NT35510_FillRect_MDMA(0, 0, LCD_W, LCD_H, COLOR_BG);
    draw_axes();

    /* 2) 正弦参数 */
    sin2t_param_t p;
    p.amplitude = (float)LCD_H * 0.4f;
    p.baseline  = (float)LCD_H * 0.5f;
    p.phase     = 0.0f;
    p.omega     = 2.0f * (float)M_PI / 200.0f;

    static uint16_t line[LCD_W];

    for (uint32_t f = 0; f < frame_count; ++f)
    {
        /* 逐行重绘（最简单直观；如需更高速，可按“条带多行”成块绘制） */
        for (int y = 0; y < LCD_H; y += TILE_LINES)
        {
            /* 行背景 */
            for (int x = 0; x < LCD_W; ++x) line[x] = COLOR_BG;

            /* 曲线 */
            for (int x = 0; x < LCD_W; ++x)
            {
                float s  = sinf(p.omega * (float)x + p.phase);
                float yy = p.baseline - p.amplitude * (s*s);
                int   yi = (int)(yy + 0.5f);

                if (yi >= 0 && yi < LCD_H && yi == y) {
                    line[x] = COLOR_CURVE;
                }
            }

            NT35510_Blit_MDMA(0, y, LCD_W, TILE_LINES, line);
        }

        /* 更新相位，制造动画 */
        p.phase += dphi_per_frame;
        if (p.phase > 2.0f * (float)M_PI) p.phase -= 2.0f * (float)M_PI;
    }
}

/* ========= 入口示例 ========= */
void Demo_Run(void)
{
    /* 保证 NT35510/FMC 已初始化 */
    // NT35510_Init();

    /* 初始化 MDMA（一次性） */
    LCD_MDMA_Init();

    /* 横屏 800x480 示例（竖屏把 LCD_W/LCD_H 对调即可） */
    Demo_Sin2t_Static();

    /* 或者：做 300 帧动画，每帧相位前进 0.08 rad */
    // Demo_Sin2t_Anim(0.08f, 300);
}
