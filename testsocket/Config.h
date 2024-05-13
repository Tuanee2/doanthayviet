#ifndef CONFIG_H
#define CONFIG_H

// cấu hình mạng và giao tiếp MQTT
const char* ssid = "Tuấn";   // tên wifi
const char* password = "manucian";  // mật khẩu wifi
const char* mqtt_server = "172.20.10.7"; // Địa chỉ IP của máy chủ Mosquitto
const int mqtt_port = 1883; // port
const char* mqtt_client_id = "ESP32Client";

// cấu hình giao tiếp RS485
#define RX_PIN 16  // Pin kết nối với A (đọc dữ liệu từ AGV)
#define TX_PIN 17  // Pin kết nối với B (gửi dữ liệu đến AGV)
#define DE_PIN 2   // Pin kết nối với chân DE (Driver Enable) của module RS485
//*************************

// cấu hình AGV
#define ORDERIDOFAGV "1234"           // thay 1234 bằng orderid của agv. 

#endif