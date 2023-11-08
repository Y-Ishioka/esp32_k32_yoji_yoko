/*
 *  Copyright(C) 2023 by Yukiya Ishioka
 */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS      5
#define TFT_RST    48
#define TFT_DC      4
#define TFT_SCLK    7
#define TFT_MOSI    6
#define TFT_CTRL   45

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);


#include "app_param.h"

/* フォントの縦方向のドット数 */
#define DEF_FONT_SIZE        32

/* フォント配列の設定 */
extern  const unsigned char  ILMH32XB_FNT[];
extern  const unsigned char  ILMZ32XB_FNT[];
extern  const unsigned char  yoji_data[];

#define DEF_FONT_A_VAR       (unsigned char *)ILMH32XB_FNT
#define DEF_FONT_K_VAR       (unsigned char *)ILMZ32XB_FNT

#define DEF_LCD_WIDTH        240
#define DEF_LCD_HIGH         320

/* 変数、配列 */
unsigned char  yoji_buffer1[ 4 ][ DEF_FONT_HIGH ][DEF_FONT_WIDTH ];
unsigned char  yoji_buffer2[ 4 ][ DEF_FONT_HIGH ][DEF_FONT_WIDTH ];
unsigned char  tmp_buffer[ DEF_FONT_HIGH ][ DEF_FONT_WIDTH ];

int  yoji_list[ DEF_YOJI_LST_MAX ];

int  lcd_char_x;
int  lcd_char_y;
int  lcd_output_flag;
int  lcd_output_color;

int  disp_color;
int  disp_color_bak;
int  yoji_item_num;
int  yoji_item_pos;
int  yoji_item_pos_bef1;
int  yoji_item_pos_bef2;

int  yoji_pos;
int  yoji_dir;
int  yoji_bef_pos;
int  yoji_bef_dir;
unsigned int  yoji_code;


uint16_t  lcd_color_tbl[] = {
  ST77XX_RED,
  ST77XX_GREEN,
  //ST77XX_BLUE,
  //ST77XX_MAGENTA,
  ST77XX_CYAN,
  ST77XX_YELLOW,
  ST77XX_WHITE,
  0x0000
};

#define  DEF_LCD_COLOR_NUM  5


#define DR_REG_RNG_BASE  0x3ff75144

/*
 *  DR_REG_RNG
 */
uint32_t getESP32Random( void )
{
  delay( 2 );
  return  READ_PERI_REG(DR_REG_RNG_BASE);
}


/*************************************************
 *  
 *************************************************/
void  y_fillScreen( uint16_t color )
{
  int  row;
  int  col;

  tft.startWrite();
  tft.setAddrWindow(0, 0, DEF_LCD_WIDTH, DEF_LCD_HIGH);
  tft.endWrite();

  for( row=0 ; row<DEF_LCD_HIGH ; row++ ) {
    for( col=0 ; col<DEF_LCD_WIDTH ; col++ ) {
      tft.pushColor( color );
    }
  }
}


/*************************************************
 *  font size  ANK
 *************************************************/
void  y_setImage_a( uint16_t x, uint16_t y, uint16_t color, unsigned char *buff )
{
  int  row;
  int  col;
  int  wx = DEF_FONT_SIZE/2;
  int  wy = DEF_FONT_SIZE;

  int  tx;
  int  ty;

  for( ty=0 ; ty<DEF_FONT_HIGH ; ty++ ) {
    for( tx=0 ; tx<DEF_FONT_WIDTH/2 ; tx++ ) {
      tmp_buffer[ tx ][ DEF_FONT_HIGH-1 - ty ] = *buff;
      buff++;
    }
  }
  buff = (unsigned char*)tmp_buffer;

  tft.startWrite();
  //tft.setAddrWindow( x, y, wx, wy );
  tft.setAddrWindow( DEF_LCD_WIDTH-1-y-wy, x, wy, wx );
  tft.endWrite();

  for( row=0 ; row<wy ; row++ ) {
    for( col=0 ; col<wx ; col++ ) {
      if( *buff != 0 ) {
        tft.pushColor( color );
      } else {
        tft.pushColor( ST77XX_BLACK );
      }
      buff++;
    }
  }
}


/*************************************************
 *  font size for KANJI
 *************************************************/
void  y_setImage_k( uint16_t x, uint16_t y, uint16_t color, unsigned char *buff )
{
  int  row;
  int  col;
  int  wx = DEF_FONT_SIZE;
  int  wy = DEF_FONT_SIZE;

  int  tx;
  int  ty;

  for( ty=0 ; ty<DEF_FONT_HIGH ; ty++ ) {
    for( tx=0 ; tx<DEF_FONT_WIDTH ; tx++ ) {
      tmp_buffer[ tx ][ DEF_FONT_HIGH-1 - ty ] = *buff;
      buff++;
    }
  }
  buff = (unsigned char*)tmp_buffer;

  tft.startWrite();
  //tft.setAddrWindow( x, y, wx, wy );
  tft.setAddrWindow( DEF_LCD_WIDTH-1-y-wy, x, wy, wx );
  tft.endWrite();

  for( row=0 ; row<wy ; row++ ) {
    for( col=0 ; col<wx ; col++ ) {
      if( *buff != 0 ) {
        tft.pushColor( color );
      } else {
        tft.pushColor( ST77XX_BLACK );
      }
      buff++;
    }
  }
}


/*************************************************
 *  
 *************************************************/
void  y_output_char_a( unsigned char *buff )
{
  int  tmp_x;
  int  tmp_y;

  tmp_x = lcd_char_x + DEF_FONT_SIZE/2;

  if( tmp_x >= DEF_LCD_WIDTH ) {
      lcd_char_x = 0;

      tmp_y = lcd_char_y + DEF_FONT_SIZE + 4;
      if( tmp_y >= DEF_LCD_HIGH ) {
          lcd_output_flag = 1;
          return;
      }
      lcd_char_y = tmp_y;
  }

  y_setImage_a( lcd_char_x, lcd_char_y, lcd_color_tbl[ lcd_output_color%DEF_LCD_COLOR_NUM ], buff );

  lcd_char_x += DEF_FONT_SIZE/2;
}


/*************************************************
 *  
 *************************************************/
void  y_output_char_k( unsigned char *buff )
{
  int  tmp_x;
  int  tmp_y;

  tmp_x = lcd_char_x + DEF_FONT_SIZE;

  if( tmp_x >= DEF_LCD_WIDTH ) {
      lcd_char_x = 0;

      tmp_y = lcd_char_y + DEF_FONT_SIZE + 4;
      if( tmp_y >= DEF_LCD_HIGH ) {
          lcd_output_flag = 1;
          return;
      }
      lcd_char_y = tmp_y;
  }

  y_setImage_k( lcd_char_x, lcd_char_y, lcd_color_tbl[ lcd_output_color%DEF_LCD_COLOR_NUM ], buff );

  lcd_char_x += DEF_FONT_SIZE;
}


/*************************************************
 *  １文字分のフォントデータを表示データ配列の指定位置へセット
 *************************************************/
void  set_font( uint8_t *font, uint8_t *buff, int width )
{
    int  i, j, k;
    int  row;
    int  w = (width/8);   /* font width byte */
    uint8_t  pat;

    /* row */
    for( i=0 ; i<DEF_FONT_HIGH ; i++ ) {
        row = DEF_FONT_WIDTH * i;
        /* col */
        for( j=0 ; j<w ; j++ ) {
            pat = 0x80;
            for( k=0 ; k<8 ; k++ ) {
                if( (font[ i * w + j ] & pat) != 0 ) {
                    buff[ row + j*8 + k ] = 1;
                }
                pat >>= 1; /* bit shift */
            }
        }
    }
}


void  make_yoji_bitmap1( unsigned char  *yoji )
{
    unsigned int  code;
    int  i;
    unsigned char  *fontdata;

	memset( yoji_buffer1, 0x00, sizeof(yoji_buffer1) );

    for( i=0 ; i<DEF_YOJI_MOJI ; i++ ) {
        code = (unsigned int)(*yoji);
        code = (code<<8) + *(yoji+1);
        fontdata = read_fontx2_k( DEF_FONT_K_VAR, code );
        set_font( fontdata, (uint8_t *)yoji_buffer1[i], DEF_FONT_WIDTH );
        yoji += 2;
    }
}


void  make_yoji_bitmap2( unsigned char  *yoji )
{
    unsigned int  code;
    int  i;
    unsigned char  *fontdata;

	memset( yoji_buffer2, 0x00, sizeof(yoji_buffer2) );

    for( i=0 ; i<DEF_YOJI_MOJI ; i++ ) {
        code = (unsigned int)(*yoji);
        code = (code<<8) + *(yoji+1);
        fontdata = read_fontx2_k( DEF_FONT_K_VAR, code );
        set_font( fontdata, (uint8_t *)yoji_buffer2[i], DEF_FONT_WIDTH );
        yoji += 2;
    }
}


void  set_font_to_lcdimg( int x, int y, unsigned char *font, unsigned int color )
{
  int  loop;
  unsigned int  ctmp;
  unsigned int  r, g, b;

  ctmp = lcd_color_tbl[color%DEF_LCD_COLOR_NUM];

  for( loop=4 ; loop>=0 ; loop-- ) {
    r = ((ctmp & 0xf800) >> loop) & 0xf800;
    g = ((ctmp & 0x07e0) >> loop) & 0x07e0;
    b = ((ctmp & 0x001f) >> loop) & 0x001f;
    //lcd_set_image( x, y, 32, 32, font, r|g|b );
    y_setImage_k( x, y, r|g|b, font );
    delay( DEF_TIM_GRAD_DLY );
  }
}


void  clr_font_to_lcdimg( int x, int y, unsigned char *font, unsigned int color )
{
  int  loop;
  unsigned int  ctmp;
  unsigned int  r, g, b;

  ctmp = lcd_color_tbl[ color%DEF_LCD_COLOR_NUM ];

  for( loop=1 ; loop<=5 ; loop++ ) {
    r = ((ctmp & 0xf800) >> loop) & 0xf800;
    g = ((ctmp & 0x07e0) >> loop) & 0x07e0;
    b = ((ctmp & 0x001f) >> loop) & 0x001f;
    if( loop == 5 ) {
      g = 0;
    }
    //lcd_set_image( x, y, 32, 32, font, r|g|b );
    y_setImage_k( x, y, r|g|b, font );
    delay( DEF_TIM_GRAD_DLY );
  }
}


/*************************************************
 *  半角文字コードからフォントデータの先頭アドレス取得
 *************************************************/
unsigned char  *read_fontx2_a( unsigned char *font, unsigned int code )
{
    unsigned char  *address = NULL ;
    unsigned int  fontbyte ;

    fontbyte = (font[14] + 7) / 8 * font[15] ;
    address = &font[17] + fontbyte * code ;

    return  address ;
}


/*************************************************
 *  全角文字コードからフォントデータの先頭アドレス取得
 *************************************************/
unsigned char  *read_fontx2_k( unsigned char *font, unsigned int code )
{
    unsigned char  *address = NULL ;
    unsigned char  *tmp ;
    unsigned int  blknum, i, fontnum ;
    unsigned int  bstart, bend ;
    unsigned int  fontbyte ;

    fontbyte = (font[14] + 7) / 8 * font[15] ;
    fontnum = 0 ;

    blknum = (unsigned int)font[17] * 4 ;
    tmp = &font[18] ;
    for( i=0 ; i<blknum ; i+=4 ) {
        bstart = tmp[i]   + ((unsigned int)tmp[i+1] << 8) ;
        bend   = tmp[i+2] + ((unsigned int)tmp[i+3] << 8) ;
        if( code >= bstart && code <= bend ) {
            address = tmp + (fontnum + (code - bstart)) * fontbyte + blknum ;
            break ;
        }

        fontnum += (bend - bstart) + 1 ;
    }

    return  address ;
}


int  yoji_item_count( void )
{
    int  count = 0;
    unsigned char  *pnt = (unsigned char *)yoji_data;

    while( *pnt != 0x00 ) {
        count++;
        pnt += 10;
    }

    return  count;
}


int  yoji_comp_list( unsigned int code, int pos )
{
    int  num;
    int  i;
    unsigned char  *pnt = (unsigned char *)yoji_data;

    for( i=0 ; i<DEF_YOJI_LST_MAX ; i++ ) {
        yoji_list[i] = 0;
    }

    pnt += pos * 2;
    num = 0;
    for( i=0 ; i<yoji_item_num ; i++ ) {
        if( *pnt == ((code >> 8) & 0xff) && *(pnt+1) == (code & 0xff) ) {
            yoji_list[ num++ ] = i;
            if( num >= DEF_YOJI_LST_MAX ) {
                break;
            }
        }
        pnt += 10;
    }

    return  num;
}


int  pos_template_y[] = {  50,  86, 122, 158 };
int  pos_template_x[] = {  80, 116, 152, 188 };


void  task1( void )
{
    unsigned char  *yoji_table;
    int  num;
    int  i;
    int  match;
    int  pos;
    int  loop;
    int  tmp_yoji_item_pos;

    yoji_pos = 0;
    yoji_dir = 0;
    yoji_bef_pos = 0;
    yoji_bef_dir = 0;
    yoji_code = 0x0000;
    yoji_item_pos_bef1 = 99999;
    yoji_item_pos_bef2 = 99999;

    yoji_item_num = yoji_item_count();
    Serial.printf( "yoji_item_num=%d", yoji_item_num );
    yoji_item_pos = rand() % yoji_item_num;
    Serial.printf( "yoji_item_pos=%d", yoji_item_pos );

    disp_color = 0;
    disp_color_bak = 0;
    yoji_table = (unsigned char *)yoji_data + yoji_item_pos * 10;

    /* set yoji */
    make_yoji_bitmap1( yoji_table );
    set_font_to_lcdimg( pos_template_x[0], pos_template_y[yoji_pos], (unsigned char *)yoji_buffer1[0], disp_color );
    set_font_to_lcdimg( pos_template_x[1], pos_template_y[yoji_pos], (unsigned char *)yoji_buffer1[1], disp_color );
    set_font_to_lcdimg( pos_template_x[2], pos_template_y[yoji_pos], (unsigned char *)yoji_buffer1[2], disp_color );
    set_font_to_lcdimg( pos_template_x[3], pos_template_y[yoji_pos], (unsigned char *)yoji_buffer1[3], disp_color );

    delay( DEF_TIM_YOJI_DLY );

    while( true ) {
        disp_color_bak = disp_color;
        disp_color++;
        if( lcd_color_tbl[disp_color] == 0x0000 ) {
            disp_color = 0;
        }

        yoji_bef_pos = yoji_pos;
        yoji_pos = rand() % DEF_YOJI_MOJI;

        /* search yoji */
        for( match=0 ; match <= 1 ; match++ ) {
            for( i=0 ; i<DEF_YOJI_MOJI ; i++ ) {
                yoji_table = (unsigned char *)yoji_data + yoji_item_pos * 10 + yoji_pos * 2;
                yoji_code = (*yoji_table << 8) + *(yoji_table + 1);

                /* make list and get list-num */
                num = yoji_comp_list( yoji_code, yoji_bef_pos );

                if( num == 0 ) {
                    Serial.printf( "yoji_pos=%d (match=%d  num=%d)  skip search.", yoji_pos, match, num );
                    yoji_pos++;
                    if( yoji_pos >= DEF_YOJI_MOJI ) {
                        yoji_pos = 0;
                    }
                    continue;
                }
                Serial.printf( "yoji_pos=%d (match=%d  num=%d)", yoji_pos, match, num );

                /* duplication check for previous data */
                tmp_yoji_item_pos = yoji_item_pos;
                for( loop=0 ; loop<5 ; loop++ ) {
                    pos = rand() % num;
                    yoji_item_pos = yoji_list[pos];
                    //if( (match > 0 && num == 1) || 
                    if( (match > 0) || 
                        (yoji_item_pos != yoji_item_pos_bef1 && yoji_item_pos != yoji_item_pos_bef2) ) {
                        match = 2;
                        break;
                    }
                }
                if( match == 2 ) {
                    break;
                } else {
                    Serial.printf( "duplicated. new=%d  bef1=%d  bef2=%d", 
                           yoji_item_pos, yoji_item_pos_bef1, yoji_item_pos_bef2 );
                    yoji_item_pos = tmp_yoji_item_pos;
                    yoji_pos++;
                    if( yoji_pos >= DEF_YOJI_MOJI ) {
                        yoji_pos = 0;
                    }
                }
            }
        }
        Serial.printf( "yoji_pos=%d  yoji_item_pos=%d", yoji_pos, yoji_item_pos );

        {
            yoji_bef_dir = yoji_dir;
            if( yoji_dir == 0 ) {
                yoji_dir = 1;
            } else {
                yoji_dir = 0;
            }

            yoji_table = (unsigned char *)yoji_data + yoji_item_pos * 10;
            /* set yoji */

            if( yoji_dir == 0 ) {
                make_yoji_bitmap1( yoji_table );
                for( i=0 ; i<4 ; i++ ) {
                  set_font_to_lcdimg( pos_template_x[i], pos_template_y[yoji_pos], (unsigned char *)yoji_buffer1[i], disp_color );
                }
            } else {
                make_yoji_bitmap2( yoji_table );
                for( i=0 ; i<4 ; i++ ) {
                  set_font_to_lcdimg( pos_template_x[yoji_pos], pos_template_y[i], (unsigned char *)yoji_buffer2[i], disp_color );
                }
            }

            delay( DEF_TIM_SET_DLY );

            if( yoji_bef_dir == 0 ) {
                for( i=0 ; i<DEF_YOJI_MOJI ; i++ ) {
                    if( i != yoji_pos ) {
                        clr_font_to_lcdimg( pos_template_x[i], pos_template_y[yoji_bef_pos], (unsigned char *)yoji_buffer1[i], disp_color_bak );
                    }
                }
            } else {
                for( i=0 ; i<DEF_YOJI_MOJI ; i++ ) {
                    if( i != yoji_pos ) {
                        clr_font_to_lcdimg( pos_template_x[yoji_bef_pos], pos_template_y[i], (unsigned char *)yoji_buffer2[i], disp_color_bak );
                    }
                }
            }

            yoji_item_pos_bef2 = yoji_item_pos_bef1;
            yoji_item_pos_bef1 = yoji_item_pos;
        }

        delay( DEF_TIM_YOJI_DLY );
    }
}


/*************************************************
 *  Arduinoの setup関数
 *************************************************/
void setup( void )
{
  /* シリアルの初期化 */
  Serial.begin(115200);
  Serial.println( "call setup()" );

  lcd_char_x = 0;
  lcd_char_y = 0;
  lcd_output_flag = 0;
  lcd_output_color = 0;

  pinMode( TFT_CTRL, OUTPUT );
  digitalWrite( TFT_CTRL, LOW );

  tft.init(240, 320);
  y_fillScreen(ST77XX_BLACK);
  delay(500);
}


/*************************************************
 *  Arduinoの loop関数
 *************************************************/
void loop( void )
{
  Serial.println( "call mainTask()" );
  srand( esp_random() );

  task1();
  delay( 100 );
}

