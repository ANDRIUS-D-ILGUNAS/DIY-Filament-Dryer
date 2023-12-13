#ifndef _DIY_DRYBOX_CFG_H_
#define _DIY_DRYBOX_CFG_H_
// clang-format off
#define MDNS_NAME                   "drybox"
#define PIN_ONE_WIRE_BUS            18
#define PIN_HEATER_CTL              16
#define PIN_FAN_PWM                 17
#define PIN_FAN_TACHO               5
#define PIN_LED_RED                 27
#define PIN_LED_GREEN               14
#define PIN_LED_BLUE                12
#define LED_ON(x)                   do{digitalWrite(x, LOW);}while(0)
#define LED_OFF(x)                  do{digitalWrite(x, HIGH);}while(0)

#define LED_ALL_OFF()               do{ digitalWrite(PIN_LED_RED, HIGH);\
                                        digitalWrite(PIN_LED_GREEN, HIGH);\
                                        digitalWrite(PIN_LED_BLUE, HIGH); }while(0)
#define LED_ALL_ON()                do{ digitalWrite(PIN_LED_RED, LOW);\
                                        digitalWrite(PIN_LED_GREEN, LOW);\
                                        digitalWrite(PIN_LED_BLUE, LOW); }while(0)
#define LED_STATUS_WIFI             PIN_LED_GREEN
#define LED_STATUS_HEATER           PIN_LED_RED
#define PID_SAMPLES                 5
#define PID_TEMP_PROXIMITY          5
#define PID_KP                      65
#define PID_KI                      81
#define PID_KD                      50

#define DISPLAY_ADDRESS             0x3C
#define HUMIDTEMP_IN_I2C_ADDRESS    0x44
#define HUMIDTEMP_OUT_I2C_ADDRESS   0x45
#define PWM_HEATER_INVERT_VALUES    1
#define PWM_FREQ_HEATER             500
#define PWM_FREQ_FAN                25000
#define PWM_RESOLUTION              10
#define PWM_MAX_VALUE               1024L
#define HEATER_DUTY_OFF             0
#define HEATER_PWM_OFF              0
#define PWM_CH_HEATER               0
#define PWM_CH_FAN                  2

#define TEMPERATURE_PRECISION       10
#define LIMIT_TEMP_IN_MIN           20
#define LIMIT_TEMP_IN_MAX           70
#define LIMIT_TEMP_HEATER_MIN       20
#define LIMIT_TEMP_HEATER_MAX       75
#define LIMIT_FAN_SPEED_MIN         0
#define LIMIT_FAN_SPEED_MAX         100

#define WIFI_CHECK_CONNECTION_MS    10000

#define DEF_USE_WEB                 1
#define DEF_DEBUG_CALIBRATE_ADC     0
#define DEF_DEBUG_PID               0
#define DEF_DEBUG_HETER_SAMPLES     0
#define DEF_DEBUG_SENSOR_SAMPLES    0
#define DEF_DEBUG_PWM_VALUES        0
#define DEF_DEBUG_DISPLAY           0
#define DEF_DEUG_SPIFFS             0

#define SHOW_HIGH_WATERMARK         0
#define TASK_DISPLAY_STACK_SIZE     4096
#define TASK_MAIN_STACK_SIZE        4096

// DISPLAY ============================
#define DISPLAY_BLINK_RATE         1000L  // ms
#define DISPLAY_FAN_RPM             200L  // ms

// FORMATTING --------------------------
#define DISPLAY_COL1                  10
#define DISPLAY_COL2                  83
// -------------------------------------


// TOP
// =====================================
// Lots of trial and error here. Lots.
// STATUS ICONS ------------------------
#define DISPLAY_ICON_WIFI_X          111
#define DISPLAY_ICON_WIFI_Y           52
#define DISPLAY_ICON_SPIFFS_X         95
#define DISPLAY_ICON_SPIFFS_Y         55
// -------------------------------------
#define DISPLAY_ICON_FAN_X             0
#define DISPLAY_ICON_FAN_Y             0

#define DISPLAY_FAN_DUTY_X            15
#define DISPLAY_FAN_DUTY_Y             9
// -------------------------------------
#define DISPLAY_INSIDE_HUMIDITY_X     50
#define DISPLAY_INSIDE_HUMIDITY_Y     20

// COLUMN 1 (Left)
// =====================================
#define DISPLAY_INSIDE_TEMP_X          DISPLAY_COL1
#define DISPLAY_INSIDE_TEMP_Y         35
#define DISPLAY_ICON_INSIDE_SENSOR_X   DISPLAY_INSIDE_TEMP_X - 10
#define DISPLAY_ICON_INSIDE_SENSOR_Y   DISPLAY_INSIDE_TEMP_Y - 10
// ------------------------------------
#define DISPLAY_HEATER_TEMP_X          DISPLAY_COL1
#define DISPLAY_HEATER_TEMP_Y         53
#define DISPLAY_ICON_HEATER_SENSOR_X   DISPLAY_HEATER_TEMP_X - 10
#define DISPLAY_ICON_HEATER_SENSOR_Y   DISPLAY_HEATER_TEMP_Y - 10
// ------------------------------------

// COLUMN 2 (Right)
// =====================================
#define DISPLAY_INSIDE_TEMP_TARGET_X      DISPLAY_COL2
#define DISPLAY_INSIDE_TEMP_TARGET_Y      DISPLAY_INSIDE_TEMP_Y -3
#define DISPLAY_ICON_INSIDE_TEMP_TARGET_X DISPLAY_INSIDE_TEMP_TARGET_X - 10
#define DISPLAY_ICON_INSIDE_TEMP_TARGET_Y DISPLAY_INSIDE_TEMP_TARGET_Y - 7
// -------------------------------------
#define DISPLAY_HEATER_TEMP_MAX_X      DISPLAY_COL2
#define DISPLAY_HEATER_TEMP_MAX_Y      DISPLAY_HEATER_TEMP_Y -3
#define DISPLAY_ICON_HEATER_TEMP_MAX_X DISPLAY_HEATER_TEMP_MAX_X - 9
#define DISPLAY_ICON_HEATER_TEMP_MAX_Y DISPLAY_HEATER_TEMP_MAX_Y - 7

// Bottom
// =====================================
#define DISPLAY_OUTSIDE_TEMP_X         DISPLAY_COL1 + 1
#define DISPLAY_OUTSIDE_TEMP_Y        64
#define DISPLAY_OUTSIDE_HUMIDITY_X     DISPLAY_OUTSIDE_TEMP_X + 40
#define DISPLAY_OUTSIDE_HUMIDITY_Y     DISPLAY_OUTSIDE_TEMP_Y
#define DISPLAY_ICON_OUTSIDE_SENSOR_X  DISPLAY_OUTSIDE_TEMP_X - 12
#define DISPLAY_ICON_OUTSIDE_SENSOR_Y  DISPLAY_OUTSIDE_TEMP_Y - 7


// clang-format on
#endif


