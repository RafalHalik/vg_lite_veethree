// /*
//  * Copyright 2019-2021, 2023 NXP
//  * All rights reserved.
//  *
//  * SPDX-License-Identifier: BSD-3-Clause
//  */

// #ifndef _DISPLAY_SUPPORT_H_
// #define _DISPLAY_SUPPORT_H_

// #include "fsl_dc_fb.h"

// // #include "display_support.h"
// #include "fsl_gpio.h"
// #include "fsl_mipi_dsi.h"

// #include "../../video/fsl_hx8394.h"

// // #include "pin_mux.h"
// // #include "board.h"
// // #include "fsl_debug_console.h"

// #if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
// #include "../../video/fsl_dc_fb_lcdifv2.h"
// #else
// #include "fsl_dc_fb_elcdif.h"
// #endif

// /*******************************************************************************
//  * Definitions
//  ******************************************************************************/

// /* @TEST_ANCHOR */
// #define DEMO_LCDIF LCDIFV2

// #define DEMO_PANEL_WIDTH  (720)    //(720)
// #define DEMO_PANEL_HEIGHT (1280)     //(1280)

// #define DEMO_HSW 6
// #define DEMO_HFP 12
// #define DEMO_HBP 24
// #define DEMO_VSW 2
// #define DEMO_VFP 16
// #define DEMO_VBP 14

// #define DEMO_PANEL_RK055AHD091 0 /* 720 * 1280, RK055AHD091-CTG(RK055HDMIPI4M) */
// #define DEMO_PANEL_RK055IQH091 1 /* 540 * 960,  RK055IQH091-CTG */
// #define DEMO_PANEL_RK055MHD091 2 /* 720 * 1280, RK055MHD091A0-CTG(RK055HDMIPI4MA0) */

// #define DEMO_DISPLAY_CONTROLLER_ELCDIF  0
// #define DEMO_DISPLAY_CONTROLLER_LCDIFV2 1

// #ifndef DEMO_PANEL
// #define DEMO_PANEL DEMO_PANEL_RK055MHD091
// #endif

// #ifndef DEMO_DISPLAY_CONTROLLER
// /* Use LCDIFV2 by default, could use ELCDIF by changing this macro. */
// #define DEMO_DISPLAY_CONTROLLER DEMO_DISPLAY_CONTROLLER_LCDIFV2
// #endif

// #define DEMO_BUFFER_FIXED_ADDRESS 0

// #if DEMO_BUFFER_FIXED_ADDRESS
// #define DEMO_BUFFER0_ADDR 0x80000000
// #define DEMO_BUFFER1_ADDR 0x80200000
// #endif

// #define DEMO_LCDIF_POL_FLAGS                                                             \
//     (kLCDIFV2_DataEnableActiveHigh | kLCDIFV2_VsyncActiveLow | kLCDIFV2_HsyncActiveLow | \
//      kLCDIFV2_DriveDataOnFallingClkEdge)

// static dc_fb_lcdifv2_handle_t s_dcFbLcdifv2Handle = {0};

// static const dc_fb_lcdifv2_config_t s_dcFbLcdifv2Config = {
//     .lcdifv2       = DEMO_LCDIF,
//     .width         = DEMO_PANEL_WIDTH,
//     .height        = DEMO_PANEL_HEIGHT,
//     .hsw           = DEMO_HSW,
//     .hfp           = DEMO_HFP,
//     .hbp           = DEMO_HBP,
//     .vsw           = DEMO_VSW,
//     .vfp           = DEMO_VFP,
//     .vbp           = DEMO_VBP,
//     .polarityFlags = DEMO_LCDIF_POL_FLAGS,
//     .lineOrder     = kLCDIFV2_LineOrderRGB,
// /* CM4 is domain 1, CM7 is domain 0. */
// #if (__CORTEX_M <= 4)
//     .domain = 1,
// #else
//     .domain = 0,
// #endif
// };

// extern const dc_fb_t g_dc;
// // const dc_fb_t g_dc = {
// //     .ops     = &g_dcFbOpsLcdifv2,
// //     .prvData = &s_dcFbLcdifv2Handle,
// //     .config  = &s_dcFbLcdifv2Config,
// // };
// /*
//  * Use the MIPI dumb panel
//  */

// /* Definitions for the frame buffer. */
// #define DEMO_BUFFER_COUNT 2 /* 2 is enough for DPI interface display. */

// #ifndef DEMO_USE_XRGB8888
// #define DEMO_USE_XRGB8888 0
// #endif

// /* Use LCDIF LUT (or named color palette) which is 8-bit per-pixel */
// #ifndef DEMO_USE_LUT8
// #define DEMO_USE_LUT8 0
// #endif

// #if DEMO_USE_XRGB8888
// #define DEMO_BUFFER_PIXEL_FORMAT   kVIDEO_PixelFormatXRGB8888
// #define DEMO_BUFFER_BYTE_PER_PIXEL 4
// #elif DEMO_USE_LUT8
// #define DEMO_BUFFER_PIXEL_FORMAT   kVIDEO_PixelFormatLUT8
// #define DEMO_BUFFER_BYTE_PER_PIXEL 1
// #else
// #define DEMO_BUFFER_PIXEL_FORMAT   kVIDEO_PixelFormatRGB565
// #define DEMO_BUFFER_BYTE_PER_PIXEL 2
// #endif

// #if ((DEMO_PANEL_RK055AHD091 == DEMO_PANEL) || (DEMO_PANEL_RK055MHD091 == DEMO_PANEL))

// #define DEMO_PANEL_WIDTH  (720)
// #define DEMO_PANEL_HEIGHT (1280)

// #elif (DEMO_PANEL_RK055IQH091 == DEMO_PANEL)

// #define DEMO_PANEL_WIDTH  (540)
// #define DEMO_PANEL_HEIGHT (960)

// #endif

// #define DEMO_BUFFER_WIDTH  DEMO_PANEL_WIDTH
// #define DEMO_BUFFER_HEIGHT DEMO_PANEL_HEIGHT

// /* Where the frame buffer is shown in the screen. */
// #define DEMO_BUFFER_START_X 0U
// #define DEMO_BUFFER_START_Y 0U

// #define DEMO_BUFFER_STRIDE_BYTE (DEMO_BUFFER_WIDTH * DEMO_BUFFER_BYTE_PER_PIXEL)
// /* There is not frame buffer aligned requirement, consider the 64-bit AXI data
//  * bus width and 32-byte cache line size, the frame buffer alignment is set to
//  * 32 byte.
//  */
// #define FRAME_BUFFER_ALIGN 32



// /*******************************************************************************
//  * API
//  ******************************************************************************/
// #if defined(__cplusplus)
// extern "C" {
// #endif /* __cplusplus */

// status_t BOARD_PrepareDisplayController(void);

// #if defined(__cplusplus)
// }
// #endif /* __cplusplus */

// #endif /* _DISPLAY_SUPPORT_H_ */
