/*
 * MqttAirSensor, yet another ESP32 based Sensor which transmits air 
 * temperature and humidity to a MQTT broker. Why? Because I can!
 *
 * Copyright (C) 2023 Julian Friedrich
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>. 
 *
 * You can file issues at https://github.com/fjulian79/MqttAirSensor/issues
 */
#include <Arduino.h>
#include <WiFi.h>
#include <math.h>
#include <rom/rtc.h>

#if defined OUTDOOR_SENSOR
#include <Adafruit_BME280.h>
#define PROJECT_CONF            "Outdoor"
#define AIRSENSOR_I2C_ADDR      0x76
#define AirSensor_t             Adafruit_BME280
#elif defined INDOOR_SENSOR
#include <Adafruit_SHT31.h>
#define PROJECT_CONF            "Indoor"
#define AIRSENSOR_I2C_ADDR
#define AirSensor_t             Adafruit_SHT31
#else
#error "Air Sensor not defined"
#endif

#include <generic/generic.hpp>
#include <generic/task.hpp>
#include <generic/uptime.hpp>
#include <param/param.hpp>
#include <cli/cli.hpp>
#include <version/version.h>
#include "mqttclient.hpp"

/**
 * The Setup Mode is intended to initialize parameters and
 * calibrate the battery.
 */
//#define SETUP_MODE              1

#define VBAT_PIN                35
#define VBAT_DEFAULT_OFFSET     153
#define VBAT_DEFAULT_NUM        1593
#define VBAT_DEN                1024
#define VBAT_MIN                3000
#define VBAT_MAX                4200

#define LED_BUILTIN_ON          digitalWrite(LED_BUILTIN, false)
#define LED_BUILTIN_OFF         digitalWrite(LED_BUILTIN, true)

#define OFFGRIDDELAY_S          10

#define DEEPSLEEP_SEC_DEFAULT   600

typedef struct
{
    struct
    {
        uint16_t offset;
        uint16_t num;

    } battery;

    struct 
    {
        char ssid[32];
        char pass[32];
    
    }wifi;

    MqttClientCfg_t mqttCfg;

    uint16_t deepsleep_s;

} Parameter_t;

typedef struct 
{
    struct
    {
        uint16_t raw;
        uint16_t mV;
        uint8_t capacity;
    } battery;

    struct 
    {
        float temperature;
        float rel_humidity;
        float abs_humidity;
        float dew_point;
#ifdef OUTDOOR_SENSOR
        float pressure;
#endif
    }air;

}GlobalData_t;

AirSensor_t AirSensor = AirSensor_t();
Param<Parameter_t> Parameter;
GlobalData_t Data;
Task ledTask(250);
Task sensorTask(5000);
Task modeSwitchTask(OFFGRIDDELAY_S * 1000, false);
Task wifiTask(30000);
UpTime upTime;
WiFiClient wifiClient;
MqttClient mqtt(wifiClient);
Cli cli;
bool networking_enabled = false;

uint16_t sample(uint8_t pin, uint16_t samples)
{
    uint32_t val = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        val += analogRead(pin);
    }

    return val/samples;
}

void readAirSensor()
{
    float a, b, gc, mw;
    float t, tk, rh;
    float svp, vp, v, dp, ah;

    t = AirSensor.readTemperature();
    rh = AirSensor.readHumidity();
    tk = t + 273.15;
    a = t >= 0 ? 7.5 : 7.6;
    b = t >= 0 ? 237.3 : 240.7;
    gc = 8314.3;
    mw = 18.016;

    svp = 6.1078 * pow(10, (a*t)/(b+t));
    vp  = rh/100 * svp;
    v = log10(vp/6.1078);
    dp = (b*v) / (a-v);
    ah = pow(10,5) * mw/gc * vp/tk;

    Data.air.temperature = t;
    Data.air.rel_humidity = rh;
    Data.air.dew_point = dp;
    Data.air.abs_humidity = ah;

    #ifdef OUTDOOR_SENSOR
    /* Sensor reading is in PA, /100 = hPA */
    Data.air.pressure = AirSensor.readPressure() / 100.0;
    #endif
}

void readVBat(uint16_t samples = 100)
{
    Data.battery.raw = sample(VBAT_PIN, samples);
    Data.battery.mV = ((Data.battery.raw + Parameter.data.battery.offset) * Parameter.data.battery.num) / VBAT_DEN;
    Data.battery.capacity = map(
        constrain(Data.battery.mV, VBAT_MIN, VBAT_MAX), 
        VBAT_MIN, VBAT_MAX, 0, 100);
}

String readString(bool secret = false)
{
    String ret;
    char c = 0;

    while(1)
    {
        while(!Serial.available());
        c = Serial.read();    

        if (c == '\r')
        {
            /* pressed return, we are done */
            Serial.write('\n');
            break;
        }
        else if (c == 0x7f || c == 0x08)
        {
            /* pressed delete */ 
            uint32_t len = ret.length();
            if (len > 0)
            {
                ret.remove(len-1);
                Serial.write("\b \b");
            }
            else
            {
                //Serial.write('\a'); 
            }
        }
        else
        {
            Serial.write(secret ? '*': c);
            ret += (char) c;
        }        
    }

    return ret;
}

void calib_vBat(void)
{
    String tmp;
    struct
    {
        uint32_t adc;
        uint32_t mv;
        
    } data[2];
    
    Serial.printf("Battery voltage sensor calibration\n");
    Serial.setTimeout(5*60000);

    for (size_t i = 0; i < 2; i++)
    {
        Serial.printf("Step %u: Provide %s battery voltage.\n", i, i == 0 ? "min." : "max.");
        Serial.printf("        Measure the voltage, enter it in mV: ");
        data[i].mv = readString().toInt();
        data[i].adc = sample(VBAT_PIN, 100);
        Serial.printf("%umV -> %u\n", data[i].mv, data[i].adc);
    }

    Parameter.data.battery.offset = (data[1].adc*data[0].mv - data[0].adc*data[1].mv) / (data[1].mv - data[0].mv);
    Parameter.data.battery.num = (data[0].mv*VBAT_DEN + VBAT_DEN/2) / (data[0].adc + Parameter.data.battery.offset);

    Serial.printf("offset: %u\n", Parameter.data.battery.offset);
    Serial.printf("scale:  %u\n", Parameter.data.battery.num);
}

void enterDeepSleep(uint32_t sleep_sec = DEEPSLEEP_SEC_DEFAULT, uint16_t delay_ms = 100)
{
    #ifdef SETUP_MODE
    Serial.printf("WARNING: Deep sleep is disabled!\n");
    #else
    /* For now there is a limmit of 2^45 usec .. respect this here */
    uint64_t usec = sleep_sec == UINT32_MAX ? (((uint64_t)0x1)<<45)-1 : sleep_sec * 1000000;

    LED_BUILTIN_OFF;

    Serial.printf("About to enter deep sleep for %d seconds.\n", sleep_sec);

    if(delay > 0)
    {
        delay(delay_ms);
    }

    ESP.deepSleep(usec);
    #endif
}

CLI_COMMAND(ver)
{
    Serial.printf("\n%s %s, Copyright (C) 2023 Julian Friedrich\n", 
            VERSION_PROJECT, VERSION_GIT_SHORT);
    Serial.printf("Config:   " PROJECT_CONF "\n");
    Serial.printf("Build:    %s, %s\n", __DATE__, __TIME__);
    Serial.printf("Git Repo: %s\n", VERSION_GIT_REMOTE_ORIGIN);
    Serial.printf("Revision: %s\n", VERSION_GIT_LONG);
    Serial.printf("\n");
    Serial.printf("This program comes with ABSOLUTELY NO WARRANTY. This is free software, and you\n");
    Serial.printf("are welcome to redistribute it under certain conditions.\n");
    Serial.printf("See GPL v3 licence at https://www.gnu.org/licenses/ for details.\n");
    Serial.printf("\n");

    return 0;
}

CLI_COMMAND(help)
{
    Serial.printf("Supported commands:\n");
    Serial.printf("  ver                Used to print version infos.\n");
    Serial.printf("  stay               Used to disable mqtt TX and deep sleep.\n");
    Serial.printf("  param cmd ...      Parameter control, see below for supported commands:\n");
    Serial.printf("    clear            Resets all values to default.\n");
    Serial.printf("    write            To paste all values at once to the terminal.\n");
    Serial.printf("    set name value   To set a single value. Valid names are:\n");
    Serial.printf("                     ssid, wifi-pass, broker-ip, broker-port,\n");
    Serial.printf("                     broker-user, broker-pass, topic, sleep,\n");
    Serial.printf("                     batt-scale, batt-offs\n");
    Serial.printf("    save             To write the parameter values to the flash.\n");
    Serial.printf("  sleep <sec>        activate deep sleep, default sec = %d.\n", DEEPSLEEP_SEC_DEFAULT);
    Serial.printf("  networking [0/1]   Disables or Enables networking at all.\n");
    Serial.printf("  i2cdetect          To detect connected i2c slaves.\n");
    Serial.printf("  reset              Used to reset the CPU.\n");
    Serial.printf("  help               Prints this text.\n"); 
    Serial.printf("\n");

    return 0;
}

CLI_COMMAND(info)
{
    Serial.printf("ESP32:\n");
    Serial.printf("  Chip:          %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("  CPU's:         %u @ %uMHz\n", ESP.getChipCores(), ESP.getCpuFreqMHz());
    Serial.printf("  Flash:         %uMB, %uMhz, Mode 0x%0x\n", ESP.getFlashChipSize()/(1024*1024), ESP.getFlashChipSpeed()/1000000, ESP.getFlashChipMode());
    Serial.printf("  PSRAM:         %u free of %u\n", ESP.getFreePsram(), ESP.getPsramSize());
    Serial.printf("  Heap:          %u free of %u\n", ESP.getFreeHeap(), ESP.getHeapSize());
    Serial.printf("  MAC:           %s\n", WiFi.macAddress().c_str());
    Serial.printf("  Reset reason:  %u\n", esp_reset_reason());
    Serial.printf("  Up time:       %s\n", upTime.toString().c_str());
    Serial.printf("\n");
    Serial.printf("Parameter:\n");
    Serial.printf("  DeepSleep:     %usec\n", Parameter.data.deepsleep_s);
    Serial.printf("  Battery:\n");
    Serial.printf("    Offset:      %u\n", Parameter.data.battery.offset);
    Serial.printf("    Scale:       %u\n", Parameter.data.battery.num);
    Serial.printf("  WiFi:\n");
    Serial.printf("    SSID:        %s\n", Parameter.data.wifi.ssid);
    Serial.printf("    Pass:        *****\n");
    Serial.printf("  MQTT:\n");
    Serial.printf("    Broker:      %s:%d\n", Parameter.data.mqttCfg.broker.ipaddr, Parameter.data.mqttCfg.broker.port);
    Serial.printf("    User:        %s\n", Parameter.data.mqttCfg.broker.user);
    Serial.printf("    Pass:        *****\n");
    Serial.printf("    Timeout:     %usec\n", mqtt.getKeepAlive());
    Serial.printf("    Topic:       %s\n", Parameter.data.mqttCfg.topic);
    Serial.printf("\n");
    Serial.printf("Sensor values:\n");
    Serial.printf("  Battery:\n");
    Serial.printf("    Volt:        %u.%03uV\n", Data.battery.mV/1000, Data.battery.mV%1000);
    Serial.printf("    Cap:         %u%%\n", Data.battery.capacity);
    Serial.printf("    Raw:         %u\n", Data.battery.raw);
    Serial.printf("  Air:\n");
    Serial.printf("    T:           %0.2f°C\n", Data.air.temperature);
    Serial.printf("    RH:          %0.2f%%\n", Data.air.rel_humidity);
    Serial.printf("    AH:          %0.2fml/m3\n", Data.air.abs_humidity);
    Serial.printf("    DP:          %0.2f°C\n", Data.air.dew_point);
#ifdef OUTDOOR_SENSOR
    Serial.printf("    AP:          %0.2fhPa\n", Data.air.pressure);
#endif
    Serial.printf("\n");
    Serial.printf("Network:\n");
    Serial.printf("  WiFi Status:   %s\n", WiFi.isConnected() ? "Connected" : "Connecting ...");
    Serial.printf("  WiFi IP:       %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  WiFi RSSI:     %d\n", WiFi.RSSI());
    Serial.printf("  MQTT Status:   %s\n", mqtt.isConnected() ? "Connected" : "Connecting ...");
    Serial.printf("\n");

    return 0;
}

CLI_COMMAND(reset)
{
    Serial.printf("Resetting the CPU ...\n");
    delay(100);
    ESP.restart();
    return 0;
}

CLI_COMMAND(networking)
{
    networking_enabled = atoi(argv[0]) == 0 ? false : true;
    Serial.printf("Networking %s\n", networking_enabled ? "on" : "off");
    return 0;
}

CLI_COMMAND(stay)
{
    modeSwitchTask.enable(false);
    Serial.printf("Off grid task disabled.\n");
    return 0;
}

CLI_COMMAND(sleep)
{
    uint16_t sleep_sec = DEEPSLEEP_SEC_DEFAULT;

    if (argc > 0)
    {
        sleep_sec = atoi(argv[0]);
    }

    enterDeepSleep(sleep_sec);
    return 0;
}

CLI_COMMAND(param)
{
    if (strcmp(argv[0], "clear") == 0)
    {
        Parameter.clear();
        Parameter.data.battery.num = VBAT_DEFAULT_NUM;
        Parameter.data.battery.offset = VBAT_DEFAULT_OFFSET;
        Serial.printf("Parameter cleared\n");
        return 0;
    }

    if (strcmp(argv[0], "write") == 0)
    {
        Serial.printf("Enter Parameters in the following order:\n");
        Serial.printf("  ssid\n");
        Serial.printf("  wifi passwd\n");
        Serial.printf("  broker ip\n");
        Serial.printf("  broker port\n");
        Serial.printf("  broker user\n");
        Serial.printf("  broker passwd\n");
        Serial.printf("  topic\n");
        Serial.printf("  deep sleep seconds\n");
        Serial.printf("  battery num\n");
        Serial.printf("  battery offset\n");
        
        readString().toCharArray(Parameter.data.wifi.ssid, 
            sizeof(Parameter.data.wifi.ssid));
        readString(true).toCharArray(Parameter.data.wifi.pass, 
            sizeof(Parameter.data.wifi.pass));
        readString().toCharArray(Parameter.data.mqttCfg.broker.ipaddr, 
            sizeof(Parameter.data.mqttCfg.broker.ipaddr));
        Parameter.data.mqttCfg.broker.port = readString().toInt();
        readString().toCharArray(Parameter.data.mqttCfg.broker.user, 
            sizeof(Parameter.data.mqttCfg.broker.user));
        readString(true).toCharArray(Parameter.data.mqttCfg.broker.pass, 
            sizeof(Parameter.data.mqttCfg.broker.pass));
        readString().toCharArray(Parameter.data.mqttCfg.topic, 
            sizeof(Parameter.data.mqttCfg.topic));
        Parameter.data.deepsleep_s = readString().toInt();
        Parameter.data.battery.num = readString().toInt();
        Parameter.data.battery.offset = readString().toInt();

        return 0;
    }

    if (strcmp(argv[0], "save") == 0)
    {
        Parameter.write();
        Serial.printf("Parameter saved\n");
        return 0;
    }

    if (strcmp(argv[0], "calib-vbat") == 0)
    {
        calib_vBat();
        return 0;
    }

    if (strcmp(argv[0], "set") == 0)
    {
        if (strcmp(argv[1], "ssid") == 0)
        {
            Serial.printf("Enter WiFi ssid: ");
            readString().toCharArray(Parameter.data.wifi.ssid, 
                sizeof(Parameter.data.wifi.ssid));
            return 0;
        }

        if (strcmp(argv[1], "wifi-pass") == 0)
        {
            Serial.printf("Enter WiFi pass: ");
            readString(true).toCharArray(Parameter.data.wifi.pass, 
                sizeof(Parameter.data.wifi.pass));
            return 0;
        }

        if (strcmp(argv[1], "broker-ip") == 0)
        {
            Serial.printf("Enter broker ip: ");
            readString().toCharArray(Parameter.data.mqttCfg.broker.ipaddr, 
                sizeof(Parameter.data.mqttCfg.broker.ipaddr));
            return 0;
        }

        if (strcmp(argv[1], "broker-port") == 0)
        {     
            Serial.printf("Enter broker port: ");
            Parameter.data.mqttCfg.broker.port = readString().toInt();
            return 0;
        }

        if (strcmp(argv[1], "broker-user") == 0)
        {
            Serial.printf("Enter broker user: ");
            readString().toCharArray(Parameter.data.mqttCfg.broker.user, 
                sizeof(Parameter.data.mqttCfg.broker.user));
            return 0;
        }

        if (strcmp(argv[1], "broker-pass") == 0)
        {
            Serial.printf("Enter broker pass: ");
            readString(true).toCharArray(Parameter.data.mqttCfg.broker.pass, 
                sizeof(Parameter.data.mqttCfg.broker.pass));
            return 0;
        }

        if (strcmp(argv[1], "topic") == 0)
        {
            Serial.printf("Enter the nodes topic: ");
            readString().toCharArray(Parameter.data.mqttCfg.topic, 
                sizeof(Parameter.data.mqttCfg.topic));
            return 0;
        }

        if (strcmp(argv[1], "sleep") == 0)
        {
            Serial.printf("Enter seconds to sleep: ");
            Parameter.data.deepsleep_s = readString().toInt();
            return 0;
        }

        if (strcmp(argv[1], "batt-scale") == 0)
        {
            Serial.printf("Enter battery scale: ");
            Parameter.data.battery.num = readString().toInt();
            return 0;
        }

        if (strcmp(argv[1], "batt-offs") == 0)
        {
            Serial.printf("Enter battery offset: ");
            Parameter.data.battery.offset= readString().toInt();
            return 0;
        }

        Serial.printf("Error: Invalid parameter!\n");
        return -1;
    }

    Serial.printf("Error: Invalid command!\n");
    return -1;
}

CLI_COMMAND(i2cdetect)
{
    Serial.printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    for(uint8_t n=0; n < 8; n++)
    {
        Serial.printf("%x0: ", n);
        for(uint8_t m=0; m < 16; m++)
        {
            uint8_t addr = (n*16) + m;
            Wire.beginTransmission(addr);
            if (Wire.endTransmission())
            {
                Serial.printf("-- ");
            }
            else
            {
                Serial.printf("%x%x ", n, m);
            }
        }
        Serial.printf("\n");
    }
    Serial.printf("\n");

    return 0;
}

void offGridTask() 
{
    String payload;

    payload = "{";
    payload += "\"battery_volt\":" + String(((float)Data.battery.mV)/1000, 3);
    payload += ",\"battery_capacity\":" + String(Data.battery.capacity);
    payload += ",\"rssi\":" + String(WiFi.RSSI());
    payload += ",\"air_temperature\":" + String(Data.air.temperature,3);
    payload += ",\"air_humidity_rel\":" + String(Data.air.rel_humidity,3);
    payload += ",\"air_humidity_abs\":" + String(Data.air.abs_humidity,3);
    payload += ",\"air_dewpoint\":" + String(Data.air.dew_point,3);
#ifdef OUTDOOR_SENSOR
    payload += ",\"air_pressure\":" + String(Data.air.pressure,3);
#endif
    payload += "}";

    if (mqtt.connect())
    {
        mqtt.publish("data", payload.c_str());
    } 

    enterDeepSleep(Parameter.data.deepsleep_s);
}

bool setup_wifi(void) 
{
    const uint8_t timeout_sec = 5;
    uint32_t start = 0;
    uint32_t ms = 0;

    Serial.printf("WiFi: Connecting to %s, ", Parameter.data.wifi.ssid);
    WiFi.mode(WIFI_STA);

    start = millis(); 
    WiFi.begin(Parameter.data.wifi.ssid, Parameter.data.wifi.pass);

    while (WiFi.status() != WL_CONNECTED)
    {
        ms = millis() - start;
        if(ms > (timeout_sec * 1000))
        {
            Serial.printf("Timeout(%u sec)\n", timeout_sec);
            return false;
        }
    }

    ms = millis() - start;
    Serial.printf("%s (%ums)\n", WiFi.localIP().toString().c_str(), ms);
    randomSeed(micros()); 
    
    mqtt.begin(Parameter.data.mqttCfg);

    return true;   
}

void setup()
{
    RESET_REASON reason = rtc_get_reset_reason(0);

    if (reason == RTCWDT_BROWN_OUT_RESET)
    {
        enterDeepSleep();
    }

    pinMode(LED_BUILTIN, OUTPUT);
    LED_BUILTIN_OFF;

    Serial.begin(115200);
    while (!Serial);   
    Serial.println();
    cmd_ver(0, 0);

    #ifdef SETUP_MODE
    Serial.printf("#########################################\n");
    Serial.printf("## WARNING: Setup Mode is active!      ##\n");
    Serial.printf("##          - Deep sleep disabled.     ##\n");
    Serial.printf("##          - Offgrid task disabled.   ##\n");
    Serial.printf("#########################################\n\n");
    #endif

    if(Parameter.begin() != true)
    {
        Serial.printf("Error: Invalid parameters.\n");
        enterDeepSleep();
    }
    mqtt.setKeepAlive(Parameter.data.deepsleep_s*2);

    readVBat();
    if (Data.battery.mV < VBAT_MIN)
    {
        Serial.printf("low Battery (%umV), going to deep sleep\n", 
            Data.battery.mV);
        enterDeepSleep();
    }

    if (!AirSensor.begin(AIRSENSOR_I2C_ADDR)) 
    {
        Serial.printf("ERROR: Air Sensor not found.\n");
#if defined OUTDOOR_SENSOR
        Serial.printf("SensorID was: 0x%x\n", AirSensor.sensorID());
        Serial.printf("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.printf("        ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.printf("        ID of 0x60 represents a BME 280.\n");
        Serial.printf("        ID of 0x61 represents a BME 680.\n");
#endif
        enterDeepSleep();
    }
    readAirSensor();

    if (Parameter.data.wifi.ssid[0] != 0 && Parameter.data.wifi.pass[0] != 0)
    {
        networking_enabled = true;
        setup_wifi();
    }

    #ifndef SETUP_MODE
    if(reason != DEEPSLEEP_RESET)
    {
        Serial.printf("Starting off grid task in %u seconds... \n", OFFGRIDDELAY_S);
        Serial.printf("Enter command 'stay' to abort.\n");
        modeSwitchTask.setLastTick(millis());
        modeSwitchTask.enable();
    }
    else
    {
        offGridTask();
    }
    #endif
    
    upTime.begin();
    cli.begin();
}

void loop()
{
    uint32_t now = millis();

    if (ledTask.isScheduled(now))
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

    if(modeSwitchTask.isScheduled(now))
    {
        LED_BUILTIN_ON;
        offGridTask();
    }

    if(sensorTask.isScheduled(now))
    {
        readAirSensor();
        readVBat();
    }

    if (networking_enabled)
    {
        if ((WiFi.status() != WL_CONNECTED) && wifiTask.isScheduled(now)) 
        {
            WiFi.disconnect();
            mqtt.disconnect();
            setup_wifi();
        }
        mqtt.loop();
    }

    upTime.loop();
    cli.loop();
}
