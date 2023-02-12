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
