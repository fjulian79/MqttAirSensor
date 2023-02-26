#ifndef _MQTTCLIENT_HPP_
#define _MQTTCLIENT_HPP_

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

typedef struct 
{
    struct broker_s
    {
        char ipaddr[32];
        uint16_t port;
        char user[32];
        char pass[32];

    }broker;

    char topic[64];

}MqttClientCfg_t;

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

class MqttClient
{
    public:

        MqttClient(Client& client);
        MqttClient(Client& client, MqttClientCfg_t& config);

        bool begin(MqttClientCfg_t& config, std::function<void(char*, uint8_t*, unsigned int)> callback=0);
        bool begin(std::function<void(char*, uint8_t*, unsigned int)> callback=0);

        void setKeepAlive(uint16_t seconds);

        bool connect(void);

        bool isConnected(void);

        void disconnect(void);

        bool publish(const char* topic, bool val, bool retain=true);

        bool publish(const char* topic, uint32_t val, bool retain=true);

        bool publish(const char* topic, float val, bool retain=true);
    
        bool publish(const char* topic, const char* data, bool retain=true);

        bool subscribe(const char* topic);

        bool loop(void);

    private:
    
        MqttClientCfg_t& Config;

        PubSubClient Mqtt;

        uint32_t LastConnect;
};

#endif /* _MQTTCLIENT_HPP_ */
