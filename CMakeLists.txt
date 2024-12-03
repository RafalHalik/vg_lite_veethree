

if(CONFIG_VG_LITE_VEETHREE)


set(ZEPHYR_CURRENT_LIBRARY vg_lite_veethree)

zephyr_interface_library_named(VG_LITE_VEETHREE)
zephyr_library()

zephyr_include_directories(vg_lite)
zephyr_include_directories(AdditionalDriver)
zephyr_include_directories(vglite)
zephyr_include_directories(include)
zephyr_include_directories(vglite/font)
zephyr_include_directories(vglite/font/mcufont/decoder)
zephyr_include_directories(vglite/VGLite/rtos)
zephyr_include_directories(vglite/VGLiteKernel)
zephyr_include_directories(vglite/VGLiteKernel/rtos)
zephyr_include_directories(video)



zephyr_library_sources(
    vglite/VGLite/vg_lite.c
    vglite/VGLite/vg_lite_image.c
    vglite/VGLite/vg_lite_matrix.c
    vglite/VGLite/vg_lite_path.c
    vglite/VGLite/nxp_support.c
    vglite/VGLite/vg_lite_flat.c
    vglite/VGLite/rtos/vg_lite_os.c
    vglite/font/buf_reader.c
    vglite/font/rle_font_read.c
    vglite/font/vft_debug.c
    vglite/font/vft_draw.c
    vglite/font/vg_lite_text.c
    vglite/font/mcufont/decoder/mf_bwfont.c
    vglite/font/mcufont/decoder/mf_encoding.c
    vglite/font/mcufont/decoder/mf_font.c
    vglite/font/mcufont/decoder/mf_justify.c
    vglite/font/mcufont/decoder/mf_kerning.c
    vglite/font/mcufont/decoder/mf_rlefont.c
    vglite/font/mcufont/decoder/mf_scaledfont.c
    vglite/font/mcufont/decoder/mf_wordwrap.c
    vglite/VGLiteKernel/vg_lite_kernel.c
    vglite/VGLiteKernel/rtos/vg_lite_hal.c
    vglite/lv_draw_vglite_arc.c 
    vglite/lv_draw_vglite_blend.c 
    vglite/lv_draw_vglite_line.c 
    vglite/lv_draw_vglite_rect.c 
    vglite/lv_draw_vglite.c 
    vglite/lv_vglite_buf.c 
    vglite/lv_vglite_utils.c
    vglite/lvgl_support.c
    vglite/vglite_support.c

    video/fsl_dc_fb_elcdif.c
    video/fsl_dc_fb_lcdifv2.c
    video/fsl_hx8394.c 
    video/fsl_mipi_dsi_cmd.c
    video/fsl_rm68191.c 
    video/fsl_rm68200.c 
    video/fsl_video_common.c

    AdditionalDriver/fsl_lcdifv2.c
)


zephyr_compile_definitions(
-DVG_COMMAND_CALL=1
-DVG_TARGET_FAST_CLEAR=0
)


zephyr_library_link_libraries(VG_LITE_VEETHREE)
target_link_libraries(VG_LITE_VEETHREE INTERFACE zephyr_interface)

endif()