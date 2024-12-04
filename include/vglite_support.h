/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _VGLITE_SUPPORT_H_
#define _VGLITE_SUPPORT_H_

#include "fsl_common.h"
#include <zephyr/kernel.h>

#include "lvgl_support.h"
// #include "lvgl.h"
#if defined(SDK_OS_FREE_RTOS)
// #include "FreeRTOS.h"
// #include "semphr.h"
#endif
// #include "board.h"
#include "/workdir/modules/hal/nxp/mcux/mcux-sdk/boards/evkmimxrt1160/board.h"

#include "fsl_gpio.h"
#include "fsl_cache.h"
#include "display_support.h"
// #include "fsl_debug_console.h"

// #include "fsl_gt911.h"
#include <zephyr/drivers/display.h>
#if LV_USE_GPU_NXP_VG_LITE
#include "vg_lite.h"
#include "vglite_support.h"
#endif

// #include "../include/vglite_support.h"

#if LV_USE_GPU_NXP_PXP
#include "draw/nxp/pxp/lv_draw_pxp_blend.h"
#endif

#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
// #include "fsl_lcdifv2.h"
#else
#include "fsl_elcdif.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif                                          /* __cplusplus */

#define VG_LITE_COMMAND_BUFFER_SIZE (256 << 10) /* 256 KB */

/* Default tessellation window width and height, in pixels */
#define DEFAULT_VG_LITE_TW_WIDTH  128 /* pixels */
#define DEFAULT_VG_LITE_TW_HEIGHT 128 /* pixels */

status_t BOARD_PrepareVGLiteController(void);

// void DEMO_FlushDisplay(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _VGLITE_SUPPORT_H_ */
