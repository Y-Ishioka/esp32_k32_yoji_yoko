◆概要

　ESP32-S3-BOX-LiteでグラフィックLCD ST7789 へ四字熟語のチェーン表示するプログラムです。

　漢字などの文字を表示するため「IPAフォント(Ver.003)」をベースとした「ILフォント」を利用しています。

　ライセンスなどは以下のファイルを参照ください。

　　IPA_Font_License_Agreement_v1.0.txt

　　README_ILM_ILG_fontx.txt

　　Readme_ipam00303.txt

■参考

ESP32-S3-BOX-Liteの３ボタンの使い方のメモです。

```  
#define ADC_SW_PIN  1

/*************************************************
 *  Buttons_ADC
 *   return : 0   no push
 *            1   left
 *            2   center
 *            3   right
 *************************************************/
int  getButtonADC( void )
{
  int  vol;
  int  btn = 0;

  vol = analogRead( ADC_SW_PIN );
  if( vol < 1200 ) {         /* 3.3/(10+3.3) -> 1016 */
    btn = 3;
  } else if( vol < 2600 ) {  /* 15/(10+15) -> 2457 */
    btn = 2;
  } else if( vol < 3200 ) {  /* 27/(10+27) -> 2988 */
    btn = 1;
  }

  return  btn;
}
```  


◆免責

　著作者は，提供するプログラムを用いることで生じるいかなる損害に関して，一切の責任を負わないものとします．

　たとえ，著作者が損害の生じる可能性を知らされていた場合も同様に一切の責任を負わないものとします．
