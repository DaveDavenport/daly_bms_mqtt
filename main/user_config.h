#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#define DALY_UART_TXD (GPIO_NUM_1)
#define DALY_UART_RXD (GPIO_NUM_2)
#define MQTT_LED (10)
#define DALY_UART_RTS (UART_PIN_NO_CHANGE)
#define DALY_UART_CTS (UART_PIN_NO_CHANGE)

#define MQTT_ROOT "house/diybattery"
const char *mqtt_topic_soc = MQTT_ROOT;
const char *mqtt_topic_mmv = MQTT_ROOT;
const char *mqtt_topic_mmt = MQTT_ROOT;
const char *mqtt_topic_cvs = MQTT_ROOT "/cells";
const char *mqtt_topic_tmps = MQTT_ROOT "/temperatures";
const char *mqtt_topic_cs = MQTT_ROOT "/state";
const char *mqtt_topic_status = MQTT_ROOT "/status";

#define WIFI_SSID "mywifi"
#define WIFI_PASSWORD "mywifipwd"
#define MQTT_URI "mqtt://mymqtt"

#endif // USER_CONFIG_H
