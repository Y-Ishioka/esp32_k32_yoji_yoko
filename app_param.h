/*
 * Copyright (C) 2022-2023 by Y.Ishioka
 */

#ifndef  _APP_PARAM_H_
#define  _APP_PARAM_H_

#define DEF_IMG_PIXEL   3
#define DEF_DISP_HIGH   64
#define DEF_IMG_WIDTH   64
#if 0
#define DEF_FONT_HIGH   16
#define DEF_FONT_WIDTH  16
#else
#define DEF_FONT_HIGH   32
#define DEF_FONT_WIDTH  32
#endif
#define IMG_HEIGHT      64

#define HEIGHT          DEF_DISP_HIGH
#define WIDTH           DEF_IMG_WIDTH

#define DEF_IMAGEBUFF   (WIDTH * HEIGHT * DEF_IMG_PIXEL)

#define DEF_YOJI_MOJI   4
#define DEF_YOJI_LST_MAX  200

#define DEF_TIM_YOJI_DLY   500
#define DEF_TIM_SET_DLY    500
#define DEF_TIM_GRAD_DLY    50
#define DEF_TIM_CLR_DLY    200

/* Item number for SD card data */
#define DEF_TEXT_BUFF   (1024)

#endif /* _APP_PARAM_H_ */

