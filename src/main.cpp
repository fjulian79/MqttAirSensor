#include <Arduino.h>
#include <WiFi.h>
#include <rom/rtc.h>
#include <Adafruit_SHT31.h>

#include <generic/generic.hpp>
#include <generic/task.hpp>
#include <generic/uptime.hpp>
#include <param/param.hpp>
#include <cli/cli.hpp>
#include <version/version.h>

#include "mqttclient.hpp"

#define VBAT_PIN                35

#define VBAT_DEFAULT_OFFSET     153
#define VBAT_DEFAULT_NUM        1593

#define VBAT_DEN                1024
#define VBAT_MIN                3000
#define VBAT_MAX                4200

#define LED_BUILTIN_ON          digitalWrite(LED_BUILTIN, false)
#define LED_BUILTIN_OFF         digitalWrite(LED_BUILTIN, true)

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
        float humidity;
    }air;

}GlobalData_t;

Adafruit_SHT31 AirSensor = Adafruit_SHT31();
Param<Parameter_t> Parameter;
GlobalData_t Data;
Task ledTask(250);
Task sensorTask(5000);
Task modeSwitchTask(10000, false);
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
    Data.air.temperature = AirSensor.readTemperature();
    Data.air.humidity = AirSensor.readHumidity();
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

void enterDeepSleep(uint32_t sleep_sec = 600, uint16_t delay_ms = 100)
{
    /* For now there is a limmit of 2^45 usec .. respect this here */
    uint64_t usec = sleep_sec == UINT32_MAX ? (((uint64_t)0x1)<<45)-1 : sleep_sec * 1000000;

    LED_BUILTIN_OFF;

    if(delay > 0)
    {
        delay(delay_ms);
    }

    ESP.deepSleep(usec);
}

CLI_COMMAND(ver)
{
    Serial.printf("\n%s %s, Copyright (C) 2023 Julian Friedrich\n", 
            VERSION_PROJECT, VERSION_GIT_SHORT);
    Serial.printf("Build:    %s, %s\n", __DATE__, __TIME__);
    Serial.printf("Git Repo: %s\n", VERSION_GIT_REMOTE_ORIGIN);
    Serial.printf("Revision: %s\n", VERSION_GIT_LONG);
    Serial.printf("\n");
    Serial.printf("This program comes with ABSOLUTELY NO WARRANTY. This is free software, and you\n");
    Serial.printf("are welcome to redistribute it under certain conditions.\n");
    Serial.printf("See GPL v3 licence at https://www.gnu.org/licenses/ for details.\n\n");

    return 0;
}

CLI_COMMAND(info)
{
    Serial.printf("ESP32:\n");
    Serial.printf("  Chip:          %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("  CPU's:         %u @ %uMHz\n", ESP.getChipCores(), ESP.getCpuFreqMHz());
    Serial.printf("  Flash:         %uMB, %uMhz, Mode 0x%0x\n", ESP.getFlashChipSize()/(1024*1024), ESP.getFlashChipSpeed()/1000000, ESP.getFlashChipMode());
    Serial.printf("  PSRAM:         %u free of %u\n", ESP.getFreePsram(), ESP.getPsramSize());
    Serial.printf("  Heap:          %u free of %u\n", ESP.getHeapSize(), ESP.getFreeHeap());
    Serial.printf("  MAC:           %s\n", WiFi.macAddress().c_str());
    Serial.printf("  Reset reason:  %u\n", esp_reset_reason());
    Serial.printf("  Up time:       %s\n", upTime.toString().c_str());
    Serial.printf("\n");
    Serial.printf("Parameter:\n");
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
    Serial.printf("    Timeout:     %u\n", Parameter.data.deepsleep_s);
    Serial.printf("    Topic:       %s\n", Parameter.data.mqttCfg.topic);
    Serial.printf("\n");
    Serial.printf("Sensor values:\n");
    Serial.printf("  Battery:       %u.%03uV, %u%%, %u\r", Data.battery.mV/1000, Data.battery.mV%1000, Data.battery.capacity, Data.battery.raw);
    Serial.printf("  Air:           %f°C, %f%%\n", Data.air.temperature, Data.air.humidity);
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
    Serial.printf("Off grid task disabled, use TBD to start it manually.\n");
    return 0;
}

CLI_COMMAND(param)
{
    if (strcmp(argv[0], "clear") == 0)
    {
        Parameter.clear();
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

void offGridTask() 
{
    String payload;

    payload = "{";
    payload += "\"battery_volt\":" + String(((float)Data.battery.mV)/1000, 3) + ",";
    payload += "\"battery_capacity\":" + String(Data.battery.capacity) + ",";
    payload += "\"rssi\":" + String(WiFi.RSSI()) + ",";
    payload += "\"air_temperature\":" + String(Data.air.temperature,3) + ",";
    payload += "\"air_humidity\":" + String(Data.air.humidity,3);
    payload += "}";

    mqtt.setKeepAlive(Parameter.data.deepsleep_s + 120);
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

    if(Parameter.begin() != true)
    {
        Serial.printf("Error: Invalid parameters.\n");
    }
    else
    {
        networking_enabled = true;
        setup_wifi();
        mqtt.begin(Parameter.data.mqttCfg);
    }

    readVBat();
    if (Data.battery.mV < VBAT_MIN)
    {
        Serial.printf("low Battery (%umV), going to deep sleep\n", 
            Data.battery.mV);
        enterDeepSleep();
    }

    if (!AirSensor.begin()) 
    {
        Serial.printf("ERROR: Air Sensor not found.\n");
        enterDeepSleep();
    }

    readAirSensor();

    if(reason != DEEPSLEEP_RESET)
    {
        Serial.printf("Starting off grid task in some seconds... \n");
        modeSwitchTask.setLastTick(millis());
        modeSwitchTask.enable();
    }
    else
    {
        offGridTask();
    }
    
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
            Serial.println("Reconnecting to WiFi...");
            WiFi.disconnect();
            WiFi.reconnect();
        }
        mqtt.loop();
    }

    upTime.loop();
    cli.loop();
}