#include "mqttclient.hpp"
#include "WiFi.h"

MqttClientCfg_t zero = {{0, 0, 0, 0}, 0};

MqttClient::MqttClient(Client& client) :
      Config(zero)
    , Mqtt(client)
    , LastConnect(0)
{
    
}

MqttClient::MqttClient(Client& client, MqttClientCfg_t& config) :
      Config(config)
    , Mqtt(client)
    , LastConnect(0)
{
    
}

bool MqttClient::begin(MqttClientCfg_t& config, std::function<void(char*, uint8_t*, unsigned int)> callback)
{
    Config = config;
    return begin(callback);
}

bool MqttClient::begin(std::function<void(char*, uint8_t*, unsigned int)> callback)
{
    if (    (strlen(Config.broker.user) == 0)
         || (strlen(Config.broker.pass) == 0)
         || (strlen(Config.topic) == 0)
         || ((uint32_t)Config.broker.ipaddr == 0) )
    {
        Serial.printf("MqttClient: Error, invalid config\n");
        return false;
    }

    Mqtt.setCallback(callback);
    return connect();
}

bool MqttClient::connect(void)
{
    String willTopic = String(Config.topic) + "/available";
    String clientId = "ESP32-" + WiFi.macAddress();
    bool status = false;

    Serial.printf("MQTT: Connecting to Broker at %s:%d, ", Config.broker.ipaddr, Config.broker.port);
    Mqtt.setServer(Config.broker.ipaddr, Config.broker.port);

    LastConnect = millis();

    status = Mqtt.connect(
        clientId.c_str(), 
        Config.broker.user, 
        Config.broker.pass, 
        willTopic.c_str(), 0, true, "offline");
    if (status) 
    {
        Serial.printf("OK.\n");
        Mqtt.publish(willTopic.c_str(), "online", true);
    }
    else 
    {
        Serial.printf("Failed, RC= %d\n", Mqtt.state());
    }

    return status;
}

bool MqttClient::isConnected(void)
{
    return Mqtt.connected();
}

void MqttClient::setKeepAlive(uint16_t seconds)
{
        Mqtt.setKeepAlive(seconds);
}

bool MqttClient::publish(const char* topic, bool val, bool retain)
{
    return publish(topic, String(val).c_str(), retain);
}

bool MqttClient::publish(const char* topic, float val, bool retain)
{
    return publish(topic, String(val, 3).c_str(), retain);
}

bool MqttClient::publish(const char* topic, uint32_t val, bool retain)
{
    return publish(topic, String(val).c_str(), retain);
}

bool MqttClient::publish(const char* topic, const char* data, bool retain)
{
    String finalTopic = String(Config.topic) + "/" + topic;
    bool status = Mqtt.connected();
    
    if(status)
    {
        status = Mqtt.publish(finalTopic.c_str(), data, retain);
    }

    Serial.printf("MQTT: %s %s at %s\n", 
        status ? "Published" : "Can't publish",
        data, finalTopic.c_str());

    return false;
}

bool MqttClient::subscribe(const char* topic)
{
    String finalTopic = String(Config.topic) + "/" + topic;

    Serial.printf("MQTT: subscribe to %s\n", finalTopic.c_str());

    return Mqtt.subscribe(finalTopic.c_str());
}

bool MqttClient::loop(void)
{
    bool connected = Mqtt.loop();

    if (!connected && (millis() - LastConnect > 10000))
    {
       connected = connect();
    }

   return connected;
}