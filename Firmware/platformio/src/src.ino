#include "Adafruit_SHT31.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "WiFi.h"
#include <DallasTemperature.h>
#include <ESPmDNS.h>
#include <OneWire.h>
#include <Wire.h>
// Local imports
#include "config.h"
#include "wifi_credentials.h"
#include "display.ino"

TaskHandle_t TaskDisplayLoop, TaskMainLoop;

OneWire oneWire(PIN_ONE_WIRE_BUS);
DallasTemperature oneWireSensors(&oneWire);
DeviceAddress heaterSensor;

AsyncWebServer server(80);
int WiFi_status = WL_IDLE_STATUS;

Adafruit_SHT31 temp_sensor_in = Adafruit_SHT31();
Adafruit_SHT31 temp_sensor_out = Adafruit_SHT31();


boolean inside_sensor_ok  = true;
boolean outside_sensor_ok = true;
boolean heater_sensor_ok  = true;
boolean wifi_ok           = true;
boolean mdns_ok           = true;
boolean spiffs_ok         = true;

float temperature_samples_in[PID_SAMPLES] = {0};
float temperature_samples_heater[PID_SAMPLES] = {0};
uint8_t ts_pos             = 0;
uint8_t fan_duty           = 0;
uint32_t pwm_raw_fan       = 0;
uint8_t heater_on          = 0;
float   heater_temp        = 0.0;
float   heater_temp_max    = 0.0;
float   inside_humidity    = 0.0;
float   inside_temp        = 0.0;
float   inside_temp_target = 0.0;
uint8_t outside_humidity   = 0;
float   outside_temp       = 0.0;

unsigned long pid_first_millis   = 0;
unsigned long wifi_tick_previous = 0;
unsigned long pid_last_millis    = 0;

#if SHOW_HIGH_WATERMARK
bool printed_highwatermark = false;
#endif

void codeForUpdatingDisplay( void * parameter ) {
  for (;;) {
#if DEF_DEBUG_DISPLAY
    Serial.print( F( "Updating display..." ) );
#endif
     display_update( heater_on
                   , inside_humidity
                   , inside_temp
                   , outside_humidity
                   , outside_temp
                   , heater_temp
                   , inside_temp_target
                   , heater_temp_max
                   , fan_duty
                   , inside_sensor_ok
                   , outside_sensor_ok
                   , heater_sensor_ok
                   , wifi_ok
                   , spiffs_ok
                   ) ;

#if DEF_DEBUG_DISPLAY
    Serial.println( F( "done" ) );
#endif
  }
}

void setup()
{
    Serial.begin(115200);

    // strcpy( ipaddr, ".000" );

    Serial.println( F( "Setting up GPIO" ) );
    pinMode(PIN_HEATER_CTL, OUTPUT);
    pinMode(PIN_FAN_PWM, OUTPUT);

    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_BLUE, OUTPUT);

    display_setup();

    LED_OFF(PIN_LED_RED);
    LED_OFF(PIN_LED_GREEN);
    LED_ON(PIN_LED_BLUE);

    // Heater PWM -- Make sure heater is initially OFF
    ledcSetup(PWM_CH_HEATER, PWM_FREQ_HEATER, PWM_RESOLUTION);
    ledcAttachPin(PIN_HEATER_CTL, PWM_CH_HEATER);
    ledcWrite(PWM_CH_HEATER, HEATER_PWM_OFF);

    // FAN PW -- Make sure FAN PWM is set to maximum (fan OFF)
    ledcSetup(PWM_CH_FAN, PWM_FREQ_FAN, PWM_RESOLUTION);
    ledcAttachPin(PIN_FAN_PWM, PWM_CH_FAN);
    ledcWrite(PWM_CH_FAN, PWM_MAX_VALUE);

    // Setup temperature and humidity oneWireSensors
    Wire.begin();
    Wire.setClock(10000);

    Serial.print( F( "Searching for SHT31 (out sensor)..." ) );
    if (!temp_sensor_out.begin(HUMIDTEMP_OUT_I2C_ADDRESS)) {
      Serial.println( F( "not found" ) );
      outside_sensor_ok = false;
    }
    else {
      Serial.println( F( "found" ) );
    }

    Serial.print( F( "Searching for SHT31 (in sensor)..." ) );
    if (!temp_sensor_in.begin(HUMIDTEMP_IN_I2C_ADDRESS)) {
      Serial.println( F( "not found" ) );
      inside_sensor_ok = false;
    }
    else {
      Serial.println( F( "found" ) );
    }

    oneWireSensors.begin();
    Serial.print( F( "Searching for address of Device 0 (heater sensor)..." ) );
    if (!oneWireSensors.getAddress(heaterSensor, 0)) {
      Serial.println( F( "not found" ) );
      heater_sensor_ok = false;
    }
    else {
      Serial.println( F( "found" ) );
    }

    // set the resolution to 9 bit (Each Dallas/Maxim device is capable of
    // several different resolutions)
    Serial.print( F( "Setting temperature precision to: " ) );
    Serial.print( TEMPERATURE_PRECISION );
    oneWireSensors.setResolution(heaterSensor, TEMPERATURE_PRECISION);
    Serial.println( F( "...done" ) );


#if DEF_USE_WEB
    delay(1000);

    // Connect to WiFi
    Serial.print( F( "Connecting to SSID: " ) );
    Serial.println( ssid );
    uint8_t attempt = 0;
    while (WiFi_status != WL_CONNECTED)
    {
        Serial.print( F( "." ) );
        WiFi_status = WiFi.begin(ssid, password);
        // wait 5 seconds and check again
        delay(2000);
        attempt ++;
        if ( attempt > 1 ){
          Serial.println( F( "failed. Continuing anyway." ) );
          break;
        }
    }
    Serial.println( F( "" ) );

    if (!MDNS.begin(MDNS_NAME)) {
        Serial.println( F( "Error starting mDNS" ) );
        mdns_ok = false;
    }
    else {
        mdns_ok = true;
        Serial.print(  F( "mDNS http://"  ) );
        Serial.print( MDNS_NAME );
        Serial.print( F( ".local" ) );
    }

    // Initialize SPIFFS
    if (!SPIFFS.begin(true))
    {
        Serial.println( F( "An Error has occurred while mounting SPIFFS" ) );
        spiffs_ok = false;
    }

    Serial.print( F( "WiFi IP: " ) );
    Serial.println(WiFi.localIP());

    // sprintf(ipaddr, "IP:%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
    // sprintf(ipaddr, "IP:.%d", WiFi.localIP()[3] );
//
    setupWebServer();
    server.begin();

#if DEF_DEBUG_SPIFFS
    debug_spiffs_files();
#endif

#endif
    Serial.println( F( "Ready to go." ) );
    LED_OFF(PIN_LED_BLUE);


    xTaskCreatePinnedToCore( codeForUpdatingDisplay  /* Task Function      */
                           , "Update_dislay"         /* Name of Task       */
                           , TASK_DISPLAY_STACK_SIZE /* Stack size of Task */
                           , NULL                    /* Parameter for Task */
                           , 1                       /* Priority of Task   */
                           , &TaskDisplayLoop        /* Task handle to keep track of Task */
                           , 1                       /* Pin to this core   */
                           );

    xTaskCreatePinnedToCore( codeForMainLoop         /* Task Function      */
                           , "Main_loop"             /* Name of Task       */
                           , TASK_MAIN_STACK_SIZE    /* Stack size of Task */
                           , NULL                    /* Parameter for Task */
                           , 1                       /* Priority of Task   */
                           , &TaskMainLoop           /* Task handle to keep track of Task */
                           , 0                       /* Pin to this core   */
                           );

}

void loop(){

  long start = millis();

#if DEF_DEBUG_TASKS
  Serial.print( F( "Finish Loop Task which runs on Core " ) );
  Serial.print( xPortGetCoreID());
  Serial.print( F( " Time " ) );
  Serial.println(millis() - start);
#endif


#if SHOW_HIGH_WATERMARK
  /* uxTaskGetStackHighWaterMark shows how much stack was never used not the
   * value of used stack. It should have to report as close as possible to 0
   * (with some margin).
   */
  if ( !printed_highwatermark ){
    int TaskMainLoopUnusedBytes = uxTaskGetStackHighWaterMark( &TaskMainLoop );
    Serial.print( F(" TaskMainLoop used " ) ) ;
    Serial.print( TASK_MAIN_STACK_SIZE - TaskMainLoopUnusedBytes );
    Serial.println( F(" bytes. " ) );

    int TaskDisplayLoopUnusedBytes = uxTaskGetStackHighWaterMark( &TaskDisplayLoop );
    Serial.print( F(" TaskDisplayLoop used " ) ) ;
    Serial.print( TASK_DISPLAY_STACK_SIZE - TaskDisplayLoopUnusedBytes );
    Serial.println( F(" bytes. " ) );
    printed_highwatermark = true;
  }
#endif


  delay(10);

}

//
// 	** Main loop **
//
void codeForMainLoop( void * parameter )
{
  for (;;) {
    // Sample temperature and humidity from all available sensors
    if (!readHeaterTemperature(&heater_temp))
    {
        // Handle heater sensor error
        Serial.println( F( "Failed to read heater temperature" ) );
        heater_sensor_ok = false;
    }
    else{
        heater_sensor_ok = true;
    }
    sample_sens_in_and_out();

// Debug
#if DEF_DEBUG_HETER_SAMPLES
    Serial.print( inside_temp );
    Serial.print( F( "\t" ) );
    Serial.print(heater_temp );
    Serial.print( F( "\t" ) );
    Serial.println( inside_temp_target );
#endif

    // Check if we have WiFi connection, if not try to reconnect
    check_wifi_connection();

    // Dry box is active
    if (heater_on)
    {
        LED_ON(LED_STATUS_HEATER);
        // Sample temperature values
        if (sample_temperatures(inside_temp, heater_temp))
        {
            // Recalculate PWM value for the heater
            heater_recalc_pwm();
        }
    }
    // Drybox is off. Turn/Keep off the heater
    else
    {
        LED_OFF(LED_STATUS_HEATER);
        set_heater_duty(HEATER_DUTY_OFF);
    }
  }
}

void debug_spiffs_files(void)
{
    File root = SPIFFS.open("/");
    File file = root.openNextFile();

    while (file)
    {
        Serial.print( F( "FILE: " ) );
        Serial.println(file.name());
        file = root.openNextFile();
    }
}

void sample_sens_in_and_out(void)
{
    float tmp_temp = 0.0;
    float tmp_humid = 0.0;
    uint8_t i = 5;

    while (i--)
    {
        tmp_temp = temp_sensor_out.readTemperature();
        // delay(5);
        tmp_humid = temp_sensor_out.readHumidity();
        if (!isnan(tmp_temp) && !isnan(tmp_humid))
        {
            break;
        }
        delay(50);
    }

    // Make sure final values are valid
    if (!isnan(tmp_temp) && !isnan(tmp_humid))
    {
        outside_temp = tmp_temp;
        outside_humidity = tmp_humid;
        outside_sensor_ok = true;
#if DEF_DEBUG_SENSOR_SAMPLES
        Serial.print( F( "TempOut:"  ) );
        Serial.print( outside_temp );
        Serial.print( F( " - HumidOut:"  ) );
        Serial.print( outside_humidity );
#endif
    }
    // Otherwise we have a problem
    else {
      Serial.println( F( "Out sensor: I2C error" ) );
      outside_sensor_ok = false;
    }

    // Read SHT temperature and humidity
    while (i--) {
        tmp_temp = temp_sensor_in.readTemperature();
        // delay(5);
        tmp_humid = temp_sensor_in.readHumidity();
        if (!isnan(tmp_temp) && !isnan(tmp_humid)) {
            break;
        }
        delay(50);
    }

    // Make sure final values are valid
    if (!isnan(tmp_temp) && !isnan(tmp_humid))
    {
        inside_temp = tmp_temp;
        inside_humidity = tmp_humid;
        inside_sensor_ok = true;
#if DEF_DEBUG_SENSOR_SAMPLES
        Serial.print( F( "TempIn:"  ) );
        Serial.print( inside_temp );
        Serial.print( F( " - HumidIn:"  ) );
        Serial.print( inside_humidity );
#endif
    }
    // Otherwise we have a problem
    else
    {
      Serial.println( F( "In sensor: I2C error" ) );
      inside_sensor_ok = false;
    }
}

bool readHeaterTemperature(float *fpTemp)
{
    float tempC;
    uint8_t i = 5;

    while (i--)
    {
        oneWireSensors.requestTemperatures();
        delay(20);
        tempC = oneWireSensors.getTempC(heaterSensor);
        if (tempC != DEVICE_DISCONNECTED_C)
        {
            *fpTemp = tempC;
            heater_sensor_ok = true;
#if DEF_DEBUG_SENSOR_SAMPLES
            Serial.print( F( "Temp C: " ) );
            Serial.println( tempC );
#endif
            return true;
        }
        else
        {
        heater_sensor_ok = false;
#if DEF_DEBUG_SENSOR_SAMPLES
            Serial.print(  F( "Heater sensor fault: "  ) );
            Serial.println( tempC );
#endif
        }
        delay(20);
    }

    return false;
}

void set_fan_duty(uint8_t duty)
{
    if (duty > 100)
    {
        duty = 100;
    }

    fan_duty = duty;
    uint32_t pwm_raw_fan = ((PWM_MAX_VALUE * (100 - duty)) / 100.0);
#if DEF_DEBUG_PWM_VALUES
    Serial.print( F( "Setting fan to: ") );
    Serial.print( pwm_raw_fan );
    Serial.print( F( " duty: ") );
    Serial.print( duty );
    Serial.print( F( " fan_duty: ") );
    Serial.print( fan_duty );
#endif
    ledcWrite(PWM_CH_FAN, pwm_raw_fan);
}

void set_heater_duty(uint8_t duty)
{
    uint32_t pwm_raw_heater = 0;
    if (duty > 100)
    {
        duty = 100;
    }

    pwm_raw_heater = ((PWM_MAX_VALUE * duty) / 100.0);

#if DEF_DEBUG_PWM_VALUES
    Serial.print( F( "Setting heater duty "  ) );
    Serial.print( duty );
    Serial.println( F( "%" ) );
#endif
    set_heater_pwm(pwm_raw_heater);
}

void set_heater_pwm(uint32_t pwm)
{
    if (pwm > PWM_MAX_VALUE)
    {
        pwm = PWM_MAX_VALUE;
    }

#if PWM_HEATER_INVERT_VALUES
    pwm = PWM_MAX_VALUE - pwm;
#endif
#if DEF_DEBUG_PWM_VALUES
    Serial.print(  F( "set_heater_duty: "  ) );
    Serial.print( pwm + "/" );
    Serial.print( F( "/"  ) );
    Serial.print( PWM_MAX_VALUE);
#endif
    ledcWrite(PWM_CH_HEATER, pwm);
}

bool sample_temperatures(float in, float heater)
{
    temperature_samples_in[ts_pos] = in;
    temperature_samples_heater[ts_pos] = heater;

    ts_pos++;
    if (ts_pos == 0) {
        // Record milliseconds of our first sample
        pid_first_millis = millis();
    }
    if (ts_pos >= PID_SAMPLES) {
        pid_last_millis = millis();
        ts_pos = 0;
        return true;
    }
    else {
        return false;
    }
}

void heater_recalc_pwm(void)
{
    // We are using a simple PID-like control loop to calculate PWM value for a
    // heater Heater is heating up our box to a target temperature and keeping
    // it steady after/if that temperature is reached. At the same time we have
    // to ensure heater does not go beyond our "safe" temperature and start
    // damaging itself/fillament/cables/enclosure etc.
    //
    // As a simple solution, we will use the following flow
    // 1. Check if heater temperature is higher or equal to the maximum set
    // heater temperature
    // 		- if True, use heater max temperature as the target for our PID
    // controller
    // 		- if False, use enclosure maximum temperature as the target for
    // our PID controller

    float average = 0;
    float pid_del_p;
    float pid_del_i;
    float pid_del_d;
    int32_t pid_val_p;
    int32_t pid_val_i;
    int32_t pid_val_d;
    int32_t pwm_val;
    float pid_target_temperature = 0.0;
    float pid_temperature = 0.0;
    float pid_temperature_previous = 0.0;
    unsigned long pid_time_diff;

    // Make sure values are valid
    if (isnan(heater_temp))
    {
        Serial.println( F( "Invalid temperature values" ) );
        Serial.print( F( "temp_heater: "  ) );
        Serial.println( heater_temp);
        Serial.print( F( "inside_temp_target: "  ) );
        Serial.println( inside_temp_target);
        set_heater_duty(HEATER_DUTY_OFF);
        return;
    }

    // Inside temperature has NOT reached the target temperature
    // However, heater temperature is approaching it's maximum allowed
    // temperature. Then, regulate heater max temperature
    if ((heater_temp >= (heater_temp_max - PID_TEMP_PROXIMITY))
         &&
        (inside_temp < inside_temp_target)
       ) {
        Serial.println( F( "Inside temperature has NOT reached the target temperature." ) );
        Serial.println( F( "However, heater temperature is approaching it's maximum allowed temp." ) );
        Serial.println( F( "Using HEATER temperature as target." ) );
        for (uint8_t i = 0; i < PID_SAMPLES; i++) {
            average += temperature_samples_heater[i];
        }
        average = average / PID_SAMPLES;

        pid_target_temperature = heater_temp_max;
        pid_temperature = heater_temp;
        pid_temperature_previous = temperature_samples_heater[PID_SAMPLES - 1];
    }
    // Heater is not close to it's maximum allowed temperature
    // Or, inside temperature has already reached the target value
    else
    {
        Serial.println( F( "Heater is not close to it's maximum allowed temperature "  ) );
        Serial.println( F( "or inside temperature has already reached the target value. " ) );
        Serial.println( F( "Using IN temperature as target." ) );
        for (uint8_t i = 0; i < PID_SAMPLES; i++) {
            average += temperature_samples_in[i];
        }
        average = average / PID_SAMPLES;

        pid_target_temperature = inside_temp_target;
        pid_temperature = inside_temp;
        pid_temperature_previous = temperature_samples_in[PID_SAMPLES - 1];
    }

    // P
    pid_del_p = pid_target_temperature - pid_temperature;
    pid_val_p = (uint32_t)(pid_del_p)*PID_KP;

    // I
    pid_del_i = pid_target_temperature - average;
    pid_val_i = (uint32_t)(pid_del_i)*PID_KI;

    // D
    pid_del_d = pid_target_temperature - pid_temperature_previous;
    pid_time_diff = pid_last_millis - pid_first_millis;
    if (pid_time_diff > 0)
    {
        pid_val_d = (uint32_t)(pid_del_d) / pid_time_diff;
        pid_val_d = (uint32_t)(pid_val_d)*PID_KD;
    }
    else
    {
        pid_val_d = 0;
    }

    // P + I + D
    pwm_val = pid_val_p + pid_val_i + pid_val_d;

#if DEF_DEBUG_PID
    Serial.print(  F( "Pd: "  ) ); Serial.println( pid_del_p );
    Serial.print(  F( "Pv: "  ) ); Serial.println( pid_val_p );
    // Serial.print(  F( "Ia: "  ) ); Serial.println( sum );
    Serial.print(  F( "Id: "  ) ); Serial.println( pid_del_i );
    Serial.print(  F( "Iv: "  ) ); Serial.println( pid_val_i );
    Serial.print(  F( "Dd: "  ) ); Serial.println( pid_del_d );
    Serial.print(  F( "Dv: "  ) ); Serial.println( pid_val_d );
    Serial.print(  F( "Calc PWM val: "  ) ); Serial.println( pwm_val );
#endif

    if (pwm_val > PWM_MAX_VALUE) {
        pwm_val = PWM_MAX_VALUE;
    }
    else if (pwm_val < 0) {
        pwm_val = 0;
    }

#if DEF_DEBUG_PID
    Serial.print(  F( "New PWM val: "  ) );
    Serial.println( pwm_val);
#endif
    set_heater_pwm(pwm_val);
}

void setupWebServer(void)
{
    server.onNotFound([](AsyncWebServerRequest *request) {
        Serial.print( F( "404: " ) );
        Serial.println(request->url());
        request->send(404);
    });

    // // send a file when /index is requested
    server.on("/index.html", HTTP_ANY, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html");
    });

    // send a file when /index is requested
    server.on("/", HTTP_ANY, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html");
    });

    server.serveStatic("/img/", SPIFFS, "/img/");
    server.serveStatic("/css/", SPIFFS, "/css/");
    server.serveStatic("/js/", SPIFFS, "/js/");
    server.serveStatic("/webfonts/", SPIFFS, "/webfonts/");

    // Get dry box status
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        char buff[200] = {0};
        int len;
        len =

            snprintf(buff, 200,
                     "{\"status\":%d, \"target_temp_in\":%f, "
                      "\"max_temp_heater\":%f, "
                      "\"temp_in\":%f, \"temp_heater\":%f, \"humid_in\":%f, "
                      "\"fan_speed\":%d, \"temp_out\":%f, \"humid_out\":%d}",
                     heater_on, inside_temp_target, heater_temp_max,
                     inside_temp, heater_temp, inside_humidity, fan_duty,
                     outside_temp, outside_humidity);
        /*
            snprintf( buff
                    , 200
                    , "{\"status\":%d, "
                      "\"target_temp_in\":%f, "
                      "\"max_temp_heater\":%f, "
                      "\"temp_in\":%f, "
                      "\"temp_heater\":%f, "
                      "\"humid_in\":%f, "
                      "\"fan_duty_percent\":%d, "
                      "\"pwm_raw_fan\":%d, "
                      "\"fan_duty\":%d, "
                      "\"temp_out\":%f, "
                      "\"humid_out\":%d"
                      "}"
                      , heater_on
                      , inside_temp_target
                      , heater_temp_max
                      , inside_temp
                      , heater_temp
                      , inside_humidity
                      , fan_duty_percent
                      , pwm_raw_fan
                      , fan_duty
                      , outside_temp
                      , outside_humidity
                    );
      */


        if (len) {
            request->send(200, "text/plain", buff);
        }
        else {
            request->send(500, "text/plain",
                          "{\"status\": \"Internal server error\"}");
        }
    });

    // Turn OFF dry box
    server.on("/off", HTTP_GET, [](AsyncWebServerRequest *request) {
        heater_on = 0;
        set_fan_duty(0);
        set_heater_duty(HEATER_DUTY_OFF);
        inside_temp_target = 0;
        heater_temp_max = 0;
        request->send(200, "text/plain", "{\"status\": \"OK\"}");
    });

    // Reboot dry box
    server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest *request) {
        heater_on = 0;
        set_fan_duty(0);
        set_heater_duty(HEATER_DUTY_OFF);
        inside_temp_target = 0;
        heater_temp_max = 0;
        request->send(200, "text/plain", "{\"status\": \"OK\"}");
        ESP.restart();
    });

    // Turn ON dry box and set target temperature, max heater temperature and fan speed
    server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request) {
        // Check if temperature, heater temperature and fan speed arguments are present
        if (  request->hasParam("temperature")
           && request->hasParam("heater")
           && request->hasParam("fanspeed")
           ) {
            String str_temperature;
            String str_heater;
            String str_fanspeed;
            int32_t temperature = 0;
            int32_t heater = 0;
            int32_t fanspeed = 0;

            str_temperature = request->getParam("temperature")->value();
            temperature = str_temperature.toInt();

            str_heater = request->getParam("heater")->value();
            heater = str_heater.toInt();

            str_fanspeed = request->getParam("fanspeed")->value();
            fanspeed = str_fanspeed.toInt();

#if DEF_DEBUG_WEB_API
            Serial.print(  F( "Target temp: "  ) );
            Serial.print(temperature );
            Serial.print( F( "°C | Heater: "  ) );
            Serial.print(heater );
            Serial.print( F( "°C | Fan Speed: "  ) );
            Serial.print(fanspeed);
#endif

            // Check drybox temperature and heater temperature limit
            if ( (temperature <= LIMIT_TEMP_IN_MAX) && (heater <= LIMIT_TEMP_HEATER_MAX) ) {
                inside_temp_target = temperature;
                heater_temp_max = heater;
                set_fan_duty(fanspeed);
                heater_on = 1;
                request->send(200, "text/plain", "{\"status\": \"OK\"}");
                return;
            }
            // If either of them are out of range, return bad request status
            else {
                request->send(400, "text/plain", "{\"status\": \"Bad request. Limit error!\"}");
                return;
            }

        }
        // Not all arguments are present in the request
        else {
            request->send(400, "text/plain", "{\"status\": \"Bad request\"}");
            return;
        }
    });
}

void check_wifi_connection(void) {
    unsigned long currentMillis = millis();

    // Check WiFi status every WIFI_CHECK_CONNECTION_MS miliseconds
    if ((currentMillis - wifi_tick_previous) >= WIFI_CHECK_CONNECTION_MS) {
        if (WiFi.status() == WL_CONNECTED) {
            LED_ON(LED_STATUS_WIFI);
            LED_OFF(PIN_LED_BLUE);
            wifi_ok  = true;
        }
        else {
            LED_OFF(LED_STATUS_WIFI);
            LED_ON(PIN_LED_BLUE);
            Serial.println( F( "Reconnecting to WiFi..." ) );
            WiFi.disconnect();
            WiFi.reconnect();
            wifi_ok  = false;
        }

        wifi_tick_previous = currentMillis;
    }
}
