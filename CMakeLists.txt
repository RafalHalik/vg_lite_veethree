
if(VG_LITE_VEETHREE)

set(ZEPHYR_CURRENT_LIBRARY vg_lite_veethree)

zephyr_interface_library_name(VG_LITE_VEETHREE)
zephyr_library()

zephyr_include_directories(vg_lite)

target_include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/vglite
    ${CMAKE_CURRENT_LIST_DIR}/vglite/inc
    ${CMAKE_CURRENT_LIST_DIR}/vglite/font
    ${CMAKE_CURRENT_LIST_DIR}/vglite/font/mcufont/decoder
    ${CMAKE_CURRENT_LIST_DIR}/vglite/VGLite/rtos
    ${CMAKE_CURRENT_LIST_DIR}/vglite/VGLiteKernel
    ${CMAKE_CURRENT_LIST_DIR}/vglite/VGLiteKernel/rtos
)

zephyr_library_sources(
    ${CMAKE_CURRENT_LIST_DIR}/vglite/VGLite/vg_lite.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/VGLite/vg_lite_image.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/VGLite/vg_lite_matrix.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/VGLite/vg_lite_path.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/VGLite/nxp_support.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/VGLite/vg_lite_flat.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/VGLite/rtos/vg_lite_os.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/font/buf_reader.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/font/rle_font_read.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/font/vft_debug.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/font/vft_draw.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/font/vg_lite_text.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/font/mcufont/decoder/mf_bwfont.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/font/mcufont/decoder/mf_encoding.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/font/mcufont/decoder/mf_font.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/font/mcufont/decoder/mf_justify.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/font/mcufont/decoder/mf_kerning.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/font/mcufont/decoder/mf_rlefont.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/font/mcufont/decoder/mf_scaledfont.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/font/mcufont/decoder/mf_wordwrap.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/VGLiteKernel/vg_lite_kernel.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/VGLiteKernel/rtos/vg_lite_hal.c
    ${CMAKE_CURRENT_LIST_DIR}/vglite/lv_draw_vglite_arc.c 
    ${CMAKE_CURRENT_LIST_DIR}/vglite/lv_draw_vglite_blend.c 
    ${CMAKE_CURRENT_LIST_DIR}/vglite/lv_draw_vglite_line.c 
    ${CMAKE_CURRENT_LIST_DIR}/vglite/lv_draw_vglite_rect.c 
    ${CMAKE_CURRENT_LIST_DIR}/vglite/lv_draw_vglite.c 
    ${CMAKE_CURRENT_LIST_DIR}/vglite/lv_vglite_buf.c 
    ${CMAKE_CURRENT_LIST_DIR}/vglite/lv_vglite_utils.c
)


target_compile_definitions(${MCUX_SDK_PROJECT_NAME} PUBLIC
-DVG_COMMAND_CALL=1
-DVG_TARGET_FAST_CLEAR=0
)


zephyr_library_link_libraries(USB_HOST_VEETHREE)
target_link_libraries(USB_HOST_VEETHREE INTERFACE zephyr_interface)

endif()