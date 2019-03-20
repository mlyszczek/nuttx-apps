/**
 * @file lv_test_theme.h
 *
 */

#ifndef LV_TEST_THEME_H
#define LV_TEST_THEME_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <graphics/lvgl.h>

#ifdef CONFIG_EXAMPLES_LVGLDEMO_THEME_1


/*********************
 *      DEFINES
 *********************/

#ifdef CONFIG_EXAMPLES_LVGLDEMO_THEME_1_HUE
#  define EXAMPLES_LVGLDEMO_THEME_1_HUE CONFIG_EXAMPLES_LVGLDEMO_THEME_1_HUE
#else
#  define EXAMPLES_LVGLDEMO_THEME_1_HUE 30
#endif

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create a test screen with a lot objects and apply the given theme on them
 * @param th pointer to a theme
 */
void lv_test_theme_1(lv_theme_t *th);

/**********************
 *      MACROS
 **********************/

#endif /*USE_LV_TESTS*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
