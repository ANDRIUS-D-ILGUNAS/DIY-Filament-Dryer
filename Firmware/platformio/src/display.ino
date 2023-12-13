#ifndef _DIY_DRYBOX_DISPLAY_INO_
#define _DIY_DRYBOX_DISPLAY_INO_

#include <Arduino.h>
#include <U8g2lib.h>

#include <SPI.h>

#include "config.h"
#include "icons.h"


void padZeros(char* charStr);

// Even though we have a 1309, the 1106 worked way better
U8G2_SH1106_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ 22);
//
// U8G2_SSD1309_128X64_NONAME0_F_4W_SW_SPI display(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
//U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
//U8G2_SSD1309_128X64_NONAME2_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
//U8G2_SSD1309_128X64_NONAME2_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
//
//
//
bool blink_heater = false;
bool blink_nospiffs = false;
char display_buf[20];
unsigned long last_millis_heater;
unsigned long last_millis_fan;
unsigned long last_millis_nospiffs;
uint8_t fan_icon = 0;
unsigned int fan_icon_timeout = 0;

// ============================================================================
// ICONS
// ============================================================================
// Core
// ----------------------------------------------------------------------------
void display_wifi_icon( bool wifi_ok ) {
  if ( wifi_ok )
    display.drawXBMP(
                      DISPLAY_ICON_WIFI_X
                    , DISPLAY_ICON_WIFI_Y
                    , DISPLAY_ICON_WIFI_OK_WIDTH
                    , DISPLAY_ICON_WIFI_OK_HEIGHT
                    , DISPLAY_ICON_WIFI_OK_BITS
                    );
  else
    display.drawXBMP(
                      DISPLAY_ICON_WIFI_X
                    , DISPLAY_ICON_WIFI_Y
                    , DISPLAY_ICON_WIFI_NOK_WIDTH
                    , DISPLAY_ICON_WIFI_NOK_HEIGHT
                    , DISPLAY_ICON_WIFI_NOK_BITS
                    );
}
void display_spiffs_icon( bool spiffs_ok ) {
  if ( !spiffs_ok )
    if (  millis() - last_millis_nospiffs >= DISPLAY_BLINK_RATE ){ // check if 1000ms passed
      last_millis_nospiffs = millis();
      blink_nospiffs = !blink_nospiffs;
    }
    if( blink_nospiffs )
      display.drawXBMP(
                        DISPLAY_ICON_SPIFFS_X
                      , DISPLAY_ICON_SPIFFS_Y
                      , DISPLAY_ICON_SPIFFS_NOK_WIDTH
                      , DISPLAY_ICON_SPIFFS_NOK_HEIGHT
                      , DISPLAY_ICON_SPIFFS_NOK_BITS
                      );
}

// Inside humidity/temp sensor
// ----------------------------------------------------------------------------
void display_icon_inside_sensor( bool inside_sensor_ok ) {
  if ( inside_sensor_ok )
    display.drawXBMP(
                      DISPLAY_ICON_INSIDE_SENSOR_X
                    , DISPLAY_ICON_INSIDE_SENSOR_Y
                    , DISPLAY_ICON_INSIDE_SENSOR_OK_WIDTH
                    , DISPLAY_ICON_INSIDE_SENSOR_OK_HEIGHT
                    , DISPLAY_ICON_INSIDE_SENSOR_OK_BITS
                    );

  else
    display.drawXBMP(
                      DISPLAY_ICON_INSIDE_SENSOR_X
                    , DISPLAY_ICON_INSIDE_SENSOR_Y
                    , DISPLAY_ICON_HUMIDTEMP_SENSOR_NOK_WIDTH
                    , DISPLAY_ICON_HUMIDTEMP_SENSOR_NOK_HEIGHT
                    , DISPLAY_ICON_HUMIDTEMP_SENSOR_NOK_BITS
                    );
}
// Outside humidity/temp sensor
// ----------------------------------------------------------------------------
void display_icon_outside_sensor( bool outside_sensor_ok ) {
  if ( outside_sensor_ok )
    display.drawXBMP(
                      DISPLAY_ICON_OUTSIDE_SENSOR_X
                    , DISPLAY_ICON_OUTSIDE_SENSOR_Y
                    , DISPLAY_ICON_OUTSIDE_SENSOR_OK_WIDTH
                    , DISPLAY_ICON_OUTSIDE_SENSOR_OK_HEIGHT
                    , DISPLAY_ICON_OUTSIDE_SENSOR_OK_BITS
                    );
  else
    display.drawXBMP(
                      DISPLAY_ICON_OUTSIDE_SENSOR_X
                    , DISPLAY_ICON_OUTSIDE_SENSOR_Y
                    , DISPLAY_ICON_HUMIDTEMP_SENSOR_NOK_WIDTH
                    , DISPLAY_ICON_HUMIDTEMP_SENSOR_NOK_HEIGHT
                    , DISPLAY_ICON_HUMIDTEMP_SENSOR_NOK_BITS
                    );
}
// Heater sensor
// ----------------------------------------------------------------------------
void display_icon_heater_sensor( bool heater_sensor_ok ) {
  if ( heater_sensor_ok )
      display.drawXBMP(
                        DISPLAY_ICON_HEATER_SENSOR_X
                      , DISPLAY_ICON_HEATER_SENSOR_Y
                      , DISPLAY_ICON_HEATER_SENSOR_OK_WIDTH
                      , DISPLAY_ICON_HEATER_SENSOR_OK_HEIGHT
                      , DISPLAY_ICON_HEATER_SENSOR_OK_BITS
                      );
  else
      display.drawXBMP(
                        DISPLAY_ICON_HEATER_SENSOR_X
                      , DISPLAY_ICON_HEATER_SENSOR_Y
                      , DISPLAY_ICON_HEATER_SENSOR_NOK_WIDTH
                      , DISPLAY_ICON_HEATER_SENSOR_NOK_HEIGHT
                      , DISPLAY_ICON_HEATER_SENSOR_NOK_BITS
                      );
}


// ============================================================================
// VALUES
// ============================================================================
void display_inside_humidity( float humidity_inside) {
  display.setFont( u8g2_font_fub20_tr );

  display.setCursor( DISPLAY_INSIDE_HUMIDITY_X, DISPLAY_INSIDE_HUMIDITY_Y );
  dtostrf( humidity_inside , 5 , 2 , display_buf);
  padZeros(display_buf);
  display.print( display_buf );
}
// ----------------------------------------------------------------------------
void display_inside_temp( float inside_temp , bool  blink_heater) {
  display.setFont( u8g2_font_fub14_tr  );

  dtostrf( inside_temp, 5, 2, display_buf );
  padZeros(display_buf);

  display.setCursor( DISPLAY_INSIDE_TEMP_X, DISPLAY_INSIDE_TEMP_Y );
  /*
  if ( blink_heater )
    display.drawXBMP(
                      DISPLAY_INSIDE_TEMP_X + 17
                    , DISPLAY_INSIDE_TEMP_Y - 15
                    , DISPLAY_ICON_HEATER_ON_WIDTH
                    , DISPLAY_ICON_HEATER_ON_HEIGHT
                    , DISPLAY_ICON_HEATER_ON_BITS
                    );
  else
  */
    display.print( display_buf );

}
// ----------------------------------------------------------------------------
void display_heater_temp( float temperature_heater ) {
  display.setFont( u8g2_font_fub14_tr  );
  display.setCursor( DISPLAY_HEATER_TEMP_X, DISPLAY_HEATER_TEMP_Y );
  dtostrf( temperature_heater, 5, 2, display_buf );
  padZeros(display_buf);
  display.print( display_buf );
}
// ----------------------------------------------------------------------------
void display_outside_temp( float temp_outside ) {
  display.setFont(u8g2_font_5x7_tf);
  dtostrf( temp_outside, 5, 2, display_buf );
  padZeros(display_buf);
  display.setCursor( DISPLAY_OUTSIDE_TEMP_X, DISPLAY_OUTSIDE_TEMP_Y );
  display.print( display_buf );
}
// ----------------------------------------------------------------------------
void display_outside_humidity( float humidity_out ) {
  display.setFont(u8g2_font_5x7_tf);
  dtostrf( humidity_out, 5, 2, display_buf );
  padZeros(display_buf);
  display.setCursor( DISPLAY_OUTSIDE_HUMIDITY_X, DISPLAY_OUTSIDE_HUMIDITY_Y );
  display.print( display_buf );
}
// ----------------------------------------------------------------------------
void display_inside_temp_target( float inside_temp_target ) {
    display.drawXBMP(
                      DISPLAY_ICON_INSIDE_TEMP_TARGET_X
                    , DISPLAY_ICON_INSIDE_TEMP_TARGET_Y
                    , DISPLAY_ICON_INSIDE_TEMP_TARGET_WIDTH
                    , DISPLAY_ICON_INSIDE_TEMP_TARGET_HEIGHT
                    , DISPLAY_ICON_INSIDE_TEMP_TARGET_BITS
                    );
    display.setFont(u8g2_font_6x10_tn);
    display.setCursor( DISPLAY_INSIDE_TEMP_TARGET_X, DISPLAY_INSIDE_TEMP_TARGET_Y );
    dtostrf( inside_temp_target, 5, 2, display_buf );
    padZeros( display_buf );
    display.print( display_buf );
}
// ----------------------------------------------------------------------------
void display_heater_temp_max( float heater_temp_max ) {
    display.drawXBMP(
                     DISPLAY_ICON_HEATER_TEMP_MAX_X
                    ,DISPLAY_ICON_HEATER_TEMP_MAX_Y
                    ,DISPLAY_ICON_HEATER_TEMP_MAX_WIDTH
                    ,DISPLAY_ICON_HEATER_TEMP_MAX_HEIGHT
                    ,DISPLAY_ICON_HEATER_TEMP_MAX_BITS
                    );
    display.setCursor( DISPLAY_HEATER_TEMP_MAX_X, DISPLAY_HEATER_TEMP_MAX_Y );
    dtostrf( heater_temp_max, 5, 2, display_buf );
    padZeros( display_buf );
    display.print( display_buf );
}
// ----------------------------------------------------------------------------
void display_fan_0(){
    display.drawXBMP(DISPLAY_ICON_FAN_X ,DISPLAY_ICON_FAN_Y
                    ,DISPLAY_ICON_FAN_00_WIDTH
                    ,DISPLAY_ICON_FAN_00_HEIGHT
                    ,DISPLAY_ICON_FAN_00_BITS
                    );}
void display_fan_1(){
    display.drawXBMP(DISPLAY_ICON_FAN_X ,DISPLAY_ICON_FAN_Y
                    ,DISPLAY_ICON_FAN_01_WIDTH
                    ,DISPLAY_ICON_FAN_01_HEIGHT
                    ,DISPLAY_ICON_FAN_01_BITS
                    );}
void display_fan_2(){
    display.drawXBMP(DISPLAY_ICON_FAN_X ,DISPLAY_ICON_FAN_Y
                    ,DISPLAY_ICON_FAN_02_WIDTH
                    ,DISPLAY_ICON_FAN_02_HEIGHT
                    ,DISPLAY_ICON_FAN_02_BITS
                    );}
void display_fan_state( uint8_t fan_duty ){
  display.setFont(u8g2_font_6x13_tr);

  display.setCursor( DISPLAY_FAN_DUTY_X, DISPLAY_FAN_DUTY_Y );
  itoa(   fan_duty, display_buf,10 );
  padZeros( display_buf );
  display.print( display_buf );

  display_fan_0();
}
void display_units(){
  display.setFont(u8g2_font_5x7_tf);
  // Humidity IN symbol
  // -----------------------------------------------------------
  display.setCursor(DISPLAY_INSIDE_HUMIDITY_X+69, DISPLAY_INSIDE_HUMIDITY_Y-14 );
  display.print("%");

  // Temperature IN symbol
  // -----------------------------------------------------------
  display.setCursor( DISPLAY_INSIDE_TEMP_X + 50, DISPLAY_INSIDE_TEMP_Y - 7);
  display.print("°C");

  // Heater Temp symbol
  // -----------------------------------------------------------
  display.setCursor( DISPLAY_HEATER_TEMP_X + 50 , DISPLAY_HEATER_TEMP_Y - 7 );
  display.print("°C");

  // Temperature OUT symbol
  // -----------------------------------------------------------
  display.setCursor( DISPLAY_OUTSIDE_TEMP_X + 24, DISPLAY_OUTSIDE_TEMP_Y - 0 );
  display.print("°C");

  // Humidity OUT symbol
  // -----------------------------------------------------------
  display.setCursor( DISPLAY_OUTSIDE_HUMIDITY_X + 25, DISPLAY_OUTSIDE_HUMIDITY_Y - 0 );
  display.print("%");

  // Fan Duty
  // -----------------------------------------------------------
  display.setCursor( DISPLAY_FAN_DUTY_X + 25, DISPLAY_FAN_DUTY_Y - 3 );
  display.print("%");
}


//=============================================================================


void display_setup() {
  display.begin();
  display.setPowerSave(0);
  display.enableUTF8Print();  // enable UTF8 support for the Arduino print()
  last_millis_heater = millis();
  last_millis_fan = millis();
}

void padZeros(char* charStr) {
  for (int i = 0; i < strlen(charStr); i++) {
    if (charStr[i]==' ')
      charStr[i]='0';
    else
      break;
  }
}


void display_update( uint8_t heater_on
                   , float   inside_humidity
                   , float   inside_temp
                   , float   outside_humidity
                   , float   outside_temp
                   , float   heater_temp
                   , float   inside_temp_target
                   , float   heater_temp_max
                   , float   fan_duty
                   , bool    inside_sensor_ok
                   , bool    outside_sensor_ok
                   , bool    heater_sensor_ok
                   , bool    wifi_ok
                   , bool    spiffs_ok
                   ) {

  int string_width;      // helper value for string widths
                         //
                       //
#if DEF_DEBUG_DISPLAY
  Serial.print( F( "millis (" ) );Serial.print( millis() );
  Serial.print( F( ") - last_millis (" ) );Serial.print( last_millis );
  Serial.print( F( ")   = " ) );Serial.println( millis() - last_millis );
  Serial.print( F("blink_heater: " ) ); Serial.println( blink_heater ? "ON" : "OFF");
  Serial.print( F("heater_on: " ) ); Serial.println( heater_on ? "ON" : "OFF");
#endif

 // Things to set if the heater is on
  if ( heater_on ){
    if (  millis() - last_millis_heater >= DISPLAY_BLINK_RATE ){ // check if 1000ms passed
      last_millis_heater = millis();
      blink_heater = !blink_heater;
    }
  }
  else
      blink_heater = false;

/*
  if ( fan_duty> 0 ){
    // The timeout is inversely proportional to the fan_duty. As the fan duty
    // goes up, the timeout goes down, and the fan icon appears to spin faster.
    fan_icon_timeout = map( fan_duty, 0, 100, 100, 1);
    if (  millis() - last_millis_fan >= fan_icon_timeout ){
      last_millis_fan = millis();
      fan_icon++;
      if ( fan_icon > 2 )
        fan_icon = 0;
    }
  }
  else{
    fan_icon = 0;
  } */


 display.firstPage();
  do {

      display_wifi_icon( wifi_ok );
      display_spiffs_icon( spiffs_ok );

      display_fan_state( fan_duty );

      display_inside_humidity( inside_humidity );

      display_icon_inside_sensor( inside_sensor_ok ) ;
      display_inside_temp( inside_temp , blink_heater );

      display_icon_heater_sensor( heater_sensor_ok ) ;
      display_heater_temp( heater_temp );

      if ( heater_on ){
        display_inside_temp_target( inside_temp_target );
        display_heater_temp_max( heater_temp_max ) ;
      }

      display_icon_outside_sensor( outside_sensor_ok ) ;
      display_outside_temp( outside_temp ) ;
      display_outside_humidity( outside_humidity ) ;

      display_units();
  } while ( display.nextPage() );
}


#endif


