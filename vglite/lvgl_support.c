/*
 * Copyright 2021-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <zephyr/kernel.h>

#include "lvgl_support.h"
#include "lvgl.h"
#if defined(SDK_OS_FREE_RTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif
#include "board.h"

#include "fsl_gpio.h"
#include "fsl_cache.h"
#include "fsl_debug_console.h"

// #include "fsl_gt911.h"
#include <zephyr/drivers/display.h>
#if LV_USE_GPU_NXP_VG_LITE
#include "vg_lite.h"
#include "vglite_support.h"
#endif

#include "../include/vglite_support.h"

#if LV_USE_GPU_NXP_PXP
#include "draw/nxp/pxp/lv_draw_pxp_blend.h"
#endif

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
#include "fsl_lcdifv2.h"
#else
#include "fsl_elcdif.h"
#endif


#define SDK_OS_FREE_RTOS 1
/*******************************************************************************
 * Definitions
 ******************************************************************************/
const dc_fb_t g_dc = {
    .ops     = &g_dcFbOpsLcdifv2,
    .prvData = &s_dcFbLcdifv2Handle,
    .config  = &s_dcFbLcdifv2Config,
};

/* Ratate panel or not. */
#ifndef DEMO_USE_ROTATE
#if LV_USE_GPU_NXP_PXP
#define DEMO_USE_ROTATE 1
#else
#define DEMO_USE_ROTATE 0
#endif
#endif

#define DEMO_MIPI_DPHY_BIT_CLK_ENLARGE(origin) (((origin) / 8) * 9)

#define DEMO_MIPI_DSI          (&g_mipiDsi)
#define DEMO_MIPI_DSI_LANE_NUM 2

/* Cache line size. */
#ifndef FSL_FEATURE_L2CACHE_LINESIZE_BYTE
#define FSL_FEATURE_L2CACHE_LINESIZE_BYTE 0
#endif
#ifndef FSL_FEATURE_L1DCACHE_LINESIZE_BYTE
#define FSL_FEATURE_L1DCACHE_LINESIZE_BYTE 0
#endif

#if (FSL_FEATURE_L2CACHE_LINESIZE_BYTE > FSL_FEATURE_L1DCACHE_LINESIZE_BYTE)
#define DEMO_CACHE_LINE_SIZE FSL_FEATURE_L2CACHE_LINESIZE_BYTE
#else
#define DEMO_CACHE_LINE_SIZE FSL_FEATURE_L1DCACHE_LINESIZE_BYTE
#endif

#if (DEMO_CACHE_LINE_SIZE > FRAME_BUFFER_ALIGN)
#define DEMO_FB_ALIGN DEMO_CACHE_LINE_SIZE
#else
#define DEMO_FB_ALIGN FRAME_BUFFER_ALIGN
#endif

#if (LV_ATTRIBUTE_MEM_ALIGN_SIZE > DEMO_FB_ALIGN)
#undef DEMO_FB_ALIGN
#define DEMO_FB_ALIGN LV_ATTRIBUTE_MEM_ALIGN_SIZE
#endif

#define DEMO_FB_SIZE \
    (((DEMO_BUFFER_WIDTH * DEMO_BUFFER_HEIGHT * LCD_FB_BYTE_PER_PIXEL) + DEMO_FB_ALIGN - 1) & ~(DEMO_FB_ALIGN - 1))

#if DEMO_USE_ROTATE
#define LVGL_BUFFER_WIDTH  DEMO_BUFFER_HEIGHT
#define LVGL_BUFFER_HEIGHT DEMO_BUFFER_WIDTH
#else
#define LVGL_BUFFER_WIDTH  DEMO_BUFFER_WIDTH
#define LVGL_BUFFER_HEIGHT DEMO_BUFFER_HEIGHT
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void DEMO_FlushDisplay(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);

#if (LV_USE_GPU_NXP_VG_LITE || LV_USE_GPU_NXP_PXP)
static void DEMO_CleanInvalidateCache(lv_disp_drv_t *disp_drv);
#endif
static void BOARD_PullPanelResetPin(bool pullUp);
static void BOARD_PullPanelPowerPin(bool pullUp);

static status_t BOARD_DSI_Transfer(dsi_transfer_t *xfer);

status_t BOARD_VerifyDisplayClockSource(void);

static void BOARD_SetMipiDsiConfig(void);

static void BOARD_InitMipiDsiClock(void);

static void DEMO_InitTouch(void);

static void DEMO_ReadTouch(lv_indev_drv_t *drv, lv_indev_data_t *data);

static void DEMO_BufferSwitchOffCallback(void *param, void *switchOffBuffer);

static void BOARD_PullMIPIPanelTouchResetPin(bool pullUp);

// static void BOARD_ConfigMIPIPanelTouchIntPin(gt911_int_pin_mode_t mode);

static void DEMO_WaitBufferSwitchOff(void);

#if ((LV_COLOR_DEPTH == 8) || (LV_COLOR_DEPTH == 1))
/*
 * To support 8 color depth and 1 color depth with this board, color palette is
 * used to map 256 color to 2^16 color.
 */
static void DEMO_SetLcdColorPalette(void);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint32_t mipiDsiTxEscClkFreq_Hz;
static uint32_t mipiDsiDphyBitClkFreq_Hz;
static uint32_t mipiDsiDphyRefClkFreq_Hz;
static uint32_t mipiDsiDpiClkFreq_Hz;

static mipi_dsi_device_t dsiDevice = {
    .virtualChannel = 0,
    .xferFunc       = BOARD_DSI_Transfer,
};


static const hx8394_resource_t hx8394Resource = {
    .dsiDevice    = &dsiDevice,
    .pullResetPin = BOARD_PullPanelResetPin,
    .pullPowerPin = BOARD_PullPanelPowerPin,
};

static display_handle_t hx8394Handle = {
    .resource = &hx8394Resource,
    .ops      = &hx8394_ops,
};

const MIPI_DSI_Type g_mipiDsi = {
    .host = DSI_HOST,
    .apb  = DSI_HOST_APB_PKT_IF,
    .dpi  = DSI_HOST_DPI_INTFC,
    .dphy = DSI_HOST_DPHY_INTFC,
};
// __attribute__((section(".sdram0")))/
SDK_ALIGN(static uint8_t  s_frameBuffer[2][DEMO_FB_SIZE], DEMO_FB_ALIGN);
#if DEMO_USE_ROTATE
SDK_ALIGN(static uint8_t s_lvglBuffer[1][DEMO_FB_SIZE], DEMO_FB_ALIGN);
#endif

#if __CORTEX_M == 4
#define DEMO_FLUSH_DCACHE() L1CACHE_CleanInvalidateSystemCache()
#else
#if DEMO_USE_ROTATE
#define DEMO_FLUSH_DCACHE() SCB_CleanInvalidateDCache_by_Addr(s_lvglBuffer[0], DEMO_FB_SIZE)
#else
#define DEMO_FLUSH_DCACHE() SCB_CleanInvalidateDCache()
#endif
#endif


static struct k_sem s_transferDone;

#if DEMO_USE_ROTATE
/*
 * When rotate is used, LVGL stack draws in one buffer (s_lvglBuffer), and LCD
 * driver uses two buffers (s_frameBuffer) to remove tearing effect.
 */
static void *volatile s_inactiveFrameBuffer;
#endif

// static gt911_handle_t s_touchHandle;
// static const gt911_config_t s_touchConfig = {
//     .I2C_SendFunc     = BOARD_MIPIPanelTouch_I2C_Send,
//     .I2C_ReceiveFunc  = BOARD_MIPIPanelTouch_I2C_Receive,
//     .pullResetPinFunc = BOARD_PullMIPIPanelTouchResetPin,
//     .intPinFunc       = BOARD_ConfigMIPIPanelTouchIntPin,
//     .timeDelayMsFunc  = VIDEO_DelayMs,
//     .touchPointNum    = 1,
//     .i2cAddrMode      = kGT911_I2cAddrMode0,
//     .intTrigMode      = kGT911_IntRisingEdge,
// };
static int s_touchResolutionX;
static int s_touchResolutionY;

// extern const dc_fb_t g_dc;

/*******************************************************************************
 * Code
 ******************************************************************************/
static status_t BOARD_DSI_Transfer(dsi_transfer_t *xfer)
{
    return DSI_TransferBlocking(DEMO_MIPI_DSI, xfer);
}


static void BOARD_PullPanelResetPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, 1);
    }
    else
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, 0);
    }
}

static void BOARD_PullPanelPowerPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, 1);
    }
    else
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, 0);
    }
}

static status_t BOARD_InitLcdPanel(void)
{
    status_t status;

    const gpio_pin_config_t pinConfig = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    const display_config_t displayConfig = {
        .resolution   = FSL_VIDEO_RESOLUTION(DEMO_PANEL_WIDTH, DEMO_PANEL_HEIGHT),
        .hsw          = DEMO_HSW,
        .hfp          = DEMO_HFP,
        .hbp          = DEMO_HBP,
        .vsw          = DEMO_VSW,
        .vfp          = DEMO_VFP,
        .vbp          = DEMO_VBP,
        .controlFlags = 0,
        .dsiLanes     = DEMO_MIPI_DSI_LANE_NUM,
    };

    GPIO_PinInit(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, &pinConfig);
    GPIO_PinInit(BOARD_MIPI_PANEL_BL_GPIO, BOARD_MIPI_PANEL_BL_PIN, &pinConfig);
    GPIO_PinInit(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, &pinConfig);

#if (DEMO_PANEL == DEMO_PANEL_RK055AHD091)
    status = RM68200_Init(&rm68200Handle, &displayConfig);

#elif (DEMO_PANEL_RK055MHD091 == DEMO_PANEL)

    status = HX8394_Init(&hx8394Handle, &displayConfig);

#else

    status = RM68191_Init(&rm68191Handle, &displayConfig);
#endif

    if (status == kStatus_Success)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_BL_GPIO, BOARD_MIPI_PANEL_BL_PIN, 1);
    }

    return status;
}

#if ((LV_COLOR_DEPTH == 8) || (LV_COLOR_DEPTH == 1))
static void DEMO_SetLcdColorPalette(void)
{
    /*
     * To support 8 color depth and 1 color depth with this board, color palette is
     * used to map 256 color to 2^16 color.
     *
     * LVGL 1-bit color depth still uses 8-bit per pixel, so the palette size is the
     * same with 8-bit color depth.
     */
    uint32_t palette[256];

#if (LV_COLOR_DEPTH == 8)
    lv_color_t color;
    color.full = 0U;

    /* RGB332 map to RGB888 */
    for (int i = 0; i < 256U; i++)
    {
        palette[i] =
            ((uint32_t)color.ch.blue << 6U) | ((uint32_t)color.ch.green << 13U) | ((uint32_t)color.ch.red << 21U);
        color.full++;
    }

#elif (LV_COLOR_DEPTH == 1)
    for (int i = 0; i < 256U;)
    {
        /*
         * Pixel map:
         * 0bXXXXXXX1 -> 0xFFFFFF
         * 0bXXXXXXX0 -> 0x000000
         */
        palette[i] = 0x000000U;
        i++;
        palette[i] = 0xFFFFFFU;
        i++;
    }
#endif

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_ELCDIF)
    ELCDIF_UpdateLut(LCDIF, kELCDIF_Lut0, 0, palette, 256);
    ELCDIF_EnableLut(LCDIF, true);
#else
    LCDIFV2_SetLut(LCDIFV2, 0, palette, 256, false);
#endif
}
#endif

void lv_port_pre_init(void)
{
}

void lv_port_disp_init(void)
{
    static lv_disp_draw_buf_t disp_buf;
    printf("%d\r\n", __LINE__);
    memset(s_frameBuffer, 0, sizeof(s_frameBuffer));
#if DEMO_USE_ROTATE
    memset(s_lvglBuffer, 0, sizeof(s_lvglBuffer));
    lv_disp_draw_buf_init(&disp_buf, s_lvglBuffer[0], NULL, DEMO_BUFFER_WIDTH * DEMO_BUFFER_HEIGHT);
#else

    lv_disp_draw_buf_init(&disp_buf, s_frameBuffer[0], s_frameBuffer[1], DEMO_BUFFER_WIDTH * DEMO_BUFFER_HEIGHT);
#endif

    status_t status;
    dc_fb_info_t fbInfo;


    /* Initialize GPU. */
    BOARD_PrepareVGLiteController();

    printf("%d %s\r\n", __LINE__, __FILE__);
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    BOARD_PrepareDisplayController();

    status = g_dc.ops->init(&g_dc);
    if (kStatus_Success != status)
    {
        assert(0);
    }

#if ((LV_COLOR_DEPTH == 8) || (LV_COLOR_DEPTH == 1))
    DEMO_SetLcdColorPalette();
#endif
 printf("%d %s\r\n", __LINE__, __FILE__);
    g_dc.ops->getLayerDefaultConfig(&g_dc, 0, &fbInfo);
    fbInfo.pixelFormat = DEMO_BUFFER_PIXEL_FORMAT;
    fbInfo.width       = DEMO_BUFFER_WIDTH;
    fbInfo.height      = DEMO_BUFFER_HEIGHT;
    fbInfo.startX      = DEMO_BUFFER_START_X;
    fbInfo.startY      = DEMO_BUFFER_START_Y;
    fbInfo.strideBytes = DEMO_BUFFER_STRIDE_BYTE;
    g_dc.ops->setLayerConfig(&g_dc, 0, &fbInfo);

    g_dc.ops->setCallback(&g_dc, 0, DEMO_BufferSwitchOffCallback, NULL);

    k_sem_init(&s_transferDone, 0 , 1);
    

#if DEMO_USE_ROTATE
    /* s_frameBuffer[1] is first shown in the panel, s_frameBuffer[0] is inactive. */
    s_inactiveFrameBuffer = (void *)s_frameBuffer[0];
#endif

    /* lvgl starts render in frame buffer 0, so show frame buffer 1 first. */
    g_dc.ops->setFrameBuffer(&g_dc, 0, (void *)s_frameBuffer[1]);
 printf("%d %s\r\n", __LINE__, __FILE__);
    /* Wait for frame buffer sent to display controller video memory. */
    if ((g_dc.ops->getProperty(&g_dc) & kDC_FB_ReserveFrameBuffer) == 0)
    {
        DEMO_WaitBufferSwitchOff();
    }

    g_dc.ops->enableLayer(&g_dc, 0);

    /*-----------------------------------
     * Register the display in LittlevGL
     *----------------------------------*/

    static lv_disp_drv_t disp_drv; /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);   /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = LVGL_BUFFER_WIDTH;
    disp_drv.ver_res = LVGL_BUFFER_HEIGHT;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = DEMO_FlushDisplay;

#if (LV_USE_GPU_NXP_VG_LITE || LV_USE_GPU_NXP_PXP)
    disp_drv.clean_dcache_cb = DEMO_CleanInvalidateCache;
#endif
 printf("%d %s\r\n", __LINE__, __FILE__);
    /*Set a display buffer*/
    disp_drv.draw_buf = &disp_buf;

    /* Partial refresh */
    disp_drv.full_refresh = 1;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
 printf("%d %s\r\n", __LINE__, __FILE__);

    if (vg_lite_init(DEFAULT_VG_LITE_TW_WIDTH, DEFAULT_VG_LITE_TW_HEIGHT) != VG_LITE_SUCCESS)
    {
        printf("%d %s\r\n", __LINE__, __FILE__);
        printf("VGLite init. FAILED. \r\n");
        vg_lite_close();
        while (1)
            ;
    }
 printf("%d %s\r\n", __LINE__, __FILE__);
    if (vg_lite_set_command_buffer_size(VG_LITE_COMMAND_BUFFER_SIZE) != VG_LITE_SUCCESS)
    {
        printf("VGLite set command buffer. FAILED. \r\n");
        vg_lite_close();
        while (1)
            ;
    }

}

static void DEMO_BufferSwitchOffCallback(void *param, void *switchOffBuffer)
{
#if defined(SDK_OS_FREE_RTOS)
    uint32_t taskAwake = 0;

    k_sem_give(&s_transferDone);
    k_yield();
#else
    s_transferDone = true;
#endif

#if DEMO_USE_ROTATE
    s_inactiveFrameBuffer = switchOffBuffer;
#endif
}

#if (LV_USE_GPU_NXP_VG_LITE || LV_USE_GPU_NXP_PXP)
static void DEMO_CleanInvalidateCache(lv_disp_drv_t *disp_drv)
{
    DEMO_FLUSH_DCACHE();
}
#endif

static void DEMO_WaitBufferSwitchOff(void)
{
#if defined(SDK_OS_FREE_RTOS)
    if (k_sem_take(&s_transferDone, K_MSEC(1)) != 0)
    {
        // printf("Display flush failed\r\n");
        // assert(0);
    }
#else
    while (false == s_transferDone)
    {
    }
    s_transferDone = false;
#endif
}

void DEMO_FlushDisplay(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
#if DEMO_USE_ROTATE

    /*
     * Work flow:
     *
     * 1. Wait for the available inactive frame buffer to draw.
     * 2. Draw the ratated frame to inactive buffer.
     * 3. Pass inactive to LCD controller to show.
     */

    static bool firstFlush = true;

    /* Only wait for the first time. */
    if (firstFlush)
    {
        firstFlush = false;
    }
    else
    {
        /* Wait frame buffer. */
        DEMO_WaitBufferSwitchOff();
    }

    /* Copy buffer. */
    void *inactiveFrameBuffer = s_inactiveFrameBuffer;
    SCB_CleanInvalidateDCache_by_Addr(inactiveFrameBuffer, DEMO_FB_SIZE);

#if LV_USE_GPU_NXP_PXP /* Use PXP to rotate the panel. */
    lv_area_t dest_area = {
        .x1 = 0,
        .x2 = DEMO_BUFFER_HEIGHT - 1,
        .y1 = 0,
        .y2 = DEMO_BUFFER_WIDTH - 1,
    };

    lv_gpu_nxp_pxp_blit(((lv_color_t *)inactiveFrameBuffer), &dest_area, DEMO_BUFFER_WIDTH, color_p, area,
                        lv_area_get_width(area), LV_OPA_COVER, LV_DISP_ROT_270);
    /* Fix the race issue between PXP and Display controller when VG_Lite is enabled */
#if LV_USE_GPU_NXP_VG_LITE
    lv_gpu_nxp_pxp_wait();
#endif

#else /* Use CPU to rotate the panel. */
    for (uint32_t y = 0; y < LVGL_BUFFER_HEIGHT; y++)
    {
        for (uint32_t x = 0; x < LVGL_BUFFER_WIDTH; x++)
        {
            ((lv_color_t *)inactiveFrameBuffer)[(DEMO_BUFFER_HEIGHT - x) * DEMO_BUFFER_WIDTH + y] =
                color_p[y * LVGL_BUFFER_WIDTH + x];
        }
    }
#endif

    SCB_CleanInvalidateDCache_by_Addr(inactiveFrameBuffer, DEMO_FB_SIZE);

    g_dc.ops->setFrameBuffer(&g_dc, 0, inactiveFrameBuffer);

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);

#else  /* DEMO_USE_ROTATE */
    SCB_CleanInvalidateDCache_by_Addr(color_p, DEMO_FB_SIZE);

    g_dc.ops->setFrameBuffer(&g_dc, 0, (void *)color_p);

    DEMO_WaitBufferSwitchOff();

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
#endif /* DEMO_USE_ROTATE */
}

void lv_port_indev_init(void)
{
    static lv_indev_drv_t indev_drv;

    /*------------------
     * Touchpad
     * -----------------*/

    /*Initialize your touchpad */
    // DEMO_InitTouch();

    /*Register a touchpad input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_POINTER;
    // indev_drv.read_cb = DEMO_ReadTouch;
    lv_indev_drv_register(&indev_drv);
}

static void BOARD_PullMIPIPanelTouchResetPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_RST_GPIO, BOARD_MIPI_PANEL_TOUCH_RST_PIN, 1);
    }
    else
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_RST_GPIO, BOARD_MIPI_PANEL_TOUCH_RST_PIN, 0);
    }
}

status_t BOARD_InitDisplayInterface(void)
{
    CLOCK_EnableClock(kCLOCK_Video_Mux);

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
    /* LCDIF v2 output to MIPI DSI. */
    VIDEO_MUX->VID_MUX_CTRL.SET = VIDEO_MUX_VID_MUX_CTRL_MIPI_DSI_SEL_MASK;
#else
    /* ELCDIF output to MIPI DSI. */
    VIDEO_MUX->VID_MUX_CTRL.CLR = VIDEO_MUX_VID_MUX_CTRL_MIPI_DSI_SEL_MASK;
#endif

    /* 1. Power on and isolation off. */
    PGMC_BPC4->BPC_POWER_CTRL |= (PGMC_BPC_BPC_POWER_CTRL_PSW_ON_SOFT_MASK | PGMC_BPC_BPC_POWER_CTRL_ISO_OFF_SOFT_MASK);

    /* 2. Assert MIPI reset. */
    IOMUXC_GPR->GPR62 &=
        ~(IOMUXC_GPR_GPR62_MIPI_DSI_PCLK_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_ESC_SOFT_RESET_N_MASK |
          IOMUXC_GPR_GPR62_MIPI_DSI_BYTE_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_DPI_SOFT_RESET_N_MASK);

    /* 3. Setup clock. */
    BOARD_InitMipiDsiClock();

    /* 4. Deassert PCLK and ESC reset. */
    IOMUXC_GPR->GPR62 |=
        (IOMUXC_GPR_GPR62_MIPI_DSI_PCLK_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_ESC_SOFT_RESET_N_MASK);

    /* 5. Configures peripheral. */
    BOARD_SetMipiDsiConfig();

    /* 6. Deassert BYTE and DBI reset. */
    IOMUXC_GPR->GPR62 |=
        (IOMUXC_GPR_GPR62_MIPI_DSI_BYTE_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_DPI_SOFT_RESET_N_MASK);

    /* 7. Configure the panel. */
    return BOARD_InitLcdPanel();
}

static void BOARD_InitLcdifClock(void)
{
    /*
     * The pixel clock is (height + VSW + VFP + VBP) * (width + HSW + HFP + HBP) * frame rate.
     *
     * For 60Hz frame rate, the RK055IQH091 pixel clock should be 36MHz.
     * the RK055AHD091 pixel clock should be 62MHz.
     */
    const clock_root_config_t lcdifClockConfig = {
        .clockOff = false,
        .mux      = 4, /*!< PLL_528. */
#if ((DEMO_PANEL == DEMO_PANEL_RK055AHD091) || (DEMO_PANEL_RK055MHD091 == DEMO_PANEL))
        .div = 9,
#else
        .div = 15,
#endif
    };

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
    CLOCK_SetRootClock(kCLOCK_Root_Lcdifv2, &lcdifClockConfig);

    mipiDsiDpiClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Lcdifv2);

#else

    CLOCK_SetRootClock(kCLOCK_Root_Lcdif, &lcdifClockConfig);

    mipiDsiDpiClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Lcdif);
#endif
}

static void BOARD_SetMipiDsiConfig(void)
{
    dsi_config_t dsiConfig;
    dsi_dphy_config_t dphyConfig;

    const dsi_dpi_config_t dpiConfig = {.pixelPayloadSize = DEMO_PANEL_WIDTH,
                                        .dpiColorCoding   = kDSI_Dpi24Bit,
                                        .pixelPacket      = kDSI_PixelPacket24Bit,
                                        .videoMode        = kDSI_DpiBurst,
                                        .bllpMode         = kDSI_DpiBllpLowPower,
                                        .polarityFlags    = kDSI_DpiVsyncActiveLow | kDSI_DpiHsyncActiveLow,
                                        .hfp              = DEMO_HFP,
                                        .hbp              = DEMO_HBP,
                                        .hsw              = DEMO_HSW,
                                        .vfp              = DEMO_VFP,
                                        .vbp              = DEMO_VBP,
                                        .panelHeight      = DEMO_PANEL_HEIGHT,
                                        .virtualChannel   = 0};

    /*
     * dsiConfig.numLanes = 4;
     * dsiConfig.enableNonContinuousHsClk = false;
     * dsiConfig.autoInsertEoTp = true;
     * dsiConfig.numExtraEoTp = 0;
     * dsiConfig.htxTo_ByteClk = 0;
     * dsiConfig.lrxHostTo_ByteClk = 0;
     * dsiConfig.btaTo_ByteClk = 0;
     */
    DSI_GetDefaultConfig(&dsiConfig);
    dsiConfig.numLanes       = DEMO_MIPI_DSI_LANE_NUM;
    dsiConfig.autoInsertEoTp = true;

    /* Init the DSI module. */
    DSI_Init(DEMO_MIPI_DSI, &dsiConfig);

    /* Init DPHY.
     *
     * The DPHY bit clock must be fast enough to send out the pixels, it should be
     * larger than:
     *
     *         (Pixel clock * bit per output pixel) / number of MIPI data lane
     *
     * Here the desired DPHY bit clock multiplied by ( 9 / 8 = 1.125) to ensure
     * it is fast enough.
     *
     * Note that the DSI output pixel is 24bit per pixel.
     */
    mipiDsiDphyBitClkFreq_Hz = mipiDsiDpiClkFreq_Hz * (24 / DEMO_MIPI_DSI_LANE_NUM);

    mipiDsiDphyBitClkFreq_Hz = DEMO_MIPI_DPHY_BIT_CLK_ENLARGE(mipiDsiDphyBitClkFreq_Hz);

    DSI_GetDphyDefaultConfig(&dphyConfig, mipiDsiDphyBitClkFreq_Hz, mipiDsiTxEscClkFreq_Hz);

    mipiDsiDphyBitClkFreq_Hz = DSI_InitDphy(DEMO_MIPI_DSI, &dphyConfig, mipiDsiDphyRefClkFreq_Hz);

    /* Init DPI interface. */
    DSI_SetDpiConfig(DEMO_MIPI_DSI, &dpiConfig, DEMO_MIPI_DSI_LANE_NUM, mipiDsiDpiClkFreq_Hz, mipiDsiDphyBitClkFreq_Hz);
}

status_t BOARD_PrepareDisplayController(void)
{
    status_t status;

    status = BOARD_VerifyDisplayClockSource();

    if (status != kStatus_Success)
    {
        printf("Error: Invalid display clock source.\r\n");
        return status;
    }

    BOARD_InitLcdifClock();

    status = BOARD_InitDisplayInterface();

    if (kStatus_Success == status)
    {
#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)

        // NVIC_ClearPendingIRQ(LCDIFv2_IRQn);
        // NVIC_SetPriority(LCDIFv2_IRQn, 3);
        // EnableIRQ(LCDIFv2_IRQn);
        // DC_FB_LCDIFV2_Init(&g_dc);
        // irq_connect_dynamic(LCDIFv2_IRQn, 2, DC_FB_LCDIFV2_IRQHandler, &g_dc, NULL);
        // irq_enable(LCDIFv2_IRQn);
#else
        NVIC_ClearPendingIRQ(eLCDIF_IRQn);
        NVIC_SetPriority(eLCDIF_IRQn, 3);
        EnableIRQ(eLCDIF_IRQn);
#endif
    }

    return kStatus_Success;
}

status_t BOARD_VerifyDisplayClockSource(void)
{
    status_t status;
    uint32_t srcClkFreq;

    /*
     * In this implementation, the SYSPLL2 (528M) clock is used as the source
     * of LCDIFV2 pixel clock and MIPI DSI ESC clock. The OSC24M clock is used
     * as the MIPI DSI DPHY PLL reference clock. This function checks the clock
     * source are valid. OSC24M is always valid, so only verify the SYSPLL2.
     */
    srcClkFreq = CLOCK_GetPllFreq(kCLOCK_PllSys2);
    if (528 != (srcClkFreq / 1000000))
    {
        status = kStatus_Fail;
    }
    else
    {
        status = kStatus_Success;
    }

    return status;
}

static void BOARD_InitMipiDsiClock(void)
{
    uint32_t mipiDsiEscClkFreq_Hz;

    /* RxClkEsc max 60MHz, TxClkEsc 12 to 20MHz. */
    /* RxClkEsc = 528MHz / 11 = 48MHz. */
    /* TxClkEsc = 528MHz / 11 / 4 = 16MHz. */
    const clock_root_config_t mipiEscClockConfig = {
        .clockOff = false,
        .mux      = 4, /*!< PLL_528. */
        .div      = 11,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Esc, &mipiEscClockConfig);

    mipiDsiEscClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Mipi_Esc);

    const clock_group_config_t mipiEscClockGroupConfig = {
        .clockOff = false, .resetDiv = 2, .div0 = 2, /* TX esc clock. */
    };

    CLOCK_SetGroupConfig(kCLOCK_Group_MipiDsi, &mipiEscClockGroupConfig);

    mipiDsiTxEscClkFreq_Hz = mipiDsiEscClkFreq_Hz / 3;

    /* DPHY reference clock, use OSC 24MHz clock. */
    const clock_root_config_t mipiDphyRefClockConfig = {
        .clockOff = false,
        .mux      = 1, /*!< OSC_24M. */
        .div      = 1,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Ref, &mipiDphyRefClockConfig);

    mipiDsiDphyRefClkFreq_Hz = BOARD_XTAL0_CLK_HZ;
}

// static void BOARD_ConfigMIPIPanelTouchIntPin(gt911_int_pin_mode_t mode)
// {
//     if (mode == kGT911_IntPinInput)
//     {
//         BOARD_MIPI_PANEL_TOUCH_INT_GPIO->GDIR &= ~(1UL << BOARD_MIPI_PANEL_TOUCH_INT_PIN);
//     }
//     else
//     {
//         if (mode == kGT911_IntPinPullDown)
//         {
//             GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_INT_GPIO, BOARD_MIPI_PANEL_TOUCH_INT_PIN, 0);
//         }
//         else
//         {
//             GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_INT_GPIO, BOARD_MIPI_PANEL_TOUCH_INT_PIN, 1);
//         }

//         BOARD_MIPI_PANEL_TOUCH_INT_GPIO->GDIR |= (1UL << BOARD_MIPI_PANEL_TOUCH_INT_PIN);
//     }
// }

/*Initialize your touchpad*/
// static void DEMO_InitTouch(void)
// {
//     status_t status;

//     const gpio_pin_config_t resetPinConfig = {
//         .direction = kGPIO_DigitalOutput, .outputLogic = 0, .interruptMode = kGPIO_NoIntmode};
//     GPIO_PinInit(BOARD_MIPI_PANEL_TOUCH_INT_GPIO, BOARD_MIPI_PANEL_TOUCH_INT_PIN, &resetPinConfig);
//     GPIO_PinInit(BOARD_MIPI_PANEL_TOUCH_RST_GPIO, BOARD_MIPI_PANEL_TOUCH_RST_PIN, &resetPinConfig);

//     status = GT911_Init(&s_touchHandle, &s_touchConfig);

//     if (kStatus_Success != status)
//     {
//         printf("Touch IC initialization failed\r\n");
//         assert(false);
//     }

//     GT911_GetResolution(&s_touchHandle, &s_touchResolutionX, &s_touchResolutionY);
// }

/* Will be called by the library to read the touchpad */
// static void DEMO_ReadTouch(lv_indev_drv_t *drv, lv_indev_data_t *data)
// {
//     static int touch_x = 0;
//     static int touch_y = 0;

//     if (kStatus_Success == GT911_GetSingleTouch(&s_touchHandle, &touch_x, &touch_y))
//     {
//         data->state = LV_INDEV_STATE_PR;
//     }
//     else
//     {
//         data->state = LV_INDEV_STATE_REL;
//     }

//     /*Set the last pressed coordinates*/
// #if DEMO_USE_ROTATE
//     data->point.x = DEMO_PANEL_HEIGHT - (touch_y * DEMO_PANEL_HEIGHT / s_touchResolutionY);
//     data->point.y = touch_x * DEMO_PANEL_WIDTH / s_touchResolutionX;
// #else
//     data->point.x = touch_x * DEMO_PANEL_WIDTH / s_touchResolutionX;
//     data->point.y = touch_y * DEMO_PANEL_HEIGHT / s_touchResolutionY;
// #endif
// }
