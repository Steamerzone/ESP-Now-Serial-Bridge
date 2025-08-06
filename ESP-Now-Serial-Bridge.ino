/*********************************************************************************
 * ESP-Now-Serial-Bridge (ESP32-C3 EWK 06-08-2025)
 *
 * ESP32 based serial bridge for transmitting serial data between two boards
 *
 * The primary purpose of this sketch was to enable a MAVLink serial connection,
 *   which I successfully tested at 57600 bps.  In theory, much faster buad rates
 *   should work fine, but I have not tested faster than 115200.
 *
 * Range is easily better than regular WiFi, however an external antenna may be
 *   required for truly long range messaging, to combat obstacles/walls, and/or
 *   to achieve success in an area saturated with 2.4GHz traffic.
 * 
 * I made heavy use of compiler macros to keep the sketch compact/efficient.
 *
 * To find the MAC address of each board, uncomment the #define DEBUG line, 
 *   and monitor serial output on boot.  Set the OPPOSITE board's IP address
 *   as the value for RECVR_MAC in the macros at the top of the sketch.
 *   
 * The BLINK_ON_* macros should be somewhat self-explanatory.  If your board has a built-in
 *   LED (or you choose to wire an external one), it can indicate ESP-Now activity as
 *   defined by the macros you choose to enable.
 *
 * When uploading the sketch, be sure to define BOARD1 or BOARD2 as appropriate
 *   before compiling.
 *
 * -- Yuri - Sep 2021
 *
 * Based this example - https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files.
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
*********************************************************************************/

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define BOARD1 // BOARD1 or BOARD2

#ifdef BOARD1
#define RECVR_MAC {0x64, 0xE8, 0x33, 0x88, 0x26, 0xF8}  // replace with BOARD2 mac address
#define MySerial Serial
#define MySerialSetup MySerial.begin(115200)            // Use USB serial
//#define BLINK_ON_SEND
#define BLINK_ON_SEND_SUCCESS
#define BLINK_ON_RECV_SUCCESS
//#define BLINK_ON_RECV

#else

#define RECVR_MAC {0x94, 0xA9, 0x90, 0x47, 0x45, 0x00}  // replace with BOARD1 mac address
HardwareSerial MySerial(1);  // Use UART1
#define TX_PIN     5                                    // connecting to pin 20/21 interferes with wifi range
#define RX_PIN     6                                    //
#define MySerialSetup MySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN)
//#define BLINK_ON_SEND
#define BLINK_ON_SEND_SUCCESS
#define BLINK_ON_RECV_SUCCESS
//#define BLINK_ON_RECV
#endif

#define WIFI_CHAN  13 // 12-13 only legal in US in lower power mode, do not use 14

#define BUFFER_SIZE 250 // max of 250 bytes
//#define DEBUG // for additional serial messages (may interfere with other messages)
#define BAUD_RATE 115200 // for calulating timeout
#define LED_BUILTIN 8  // on-board led pin number

const uint8_t broadcastAddress[] = RECVR_MAC;
// wait for double the time between bytes at this serial baud rate before sending a packet
// this *should* allow for complete packet forming when using packetized serial comms
const uint32_t timeout_micros = (int)(1.0 / BAUD_RATE * 1E6) * 20;

uint8_t buf_recv[BUFFER_SIZE];
uint8_t buf_send[BUFFER_SIZE];
uint8_t buf_size = 0;
uint32_t send_timeout = 0;
uint8_t led_status = 0;

esp_now_peer_info_t peerInfo;  // scope workaround for arduino-esp32 v2.0.1

#if defined(DEBUG) || defined(BLINK_ON_SEND_SUCCESS)
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  #ifdef DEBUG
  if (status == ESP_NOW_SEND_SUCCESS) {
    MySerial.println("Send success");
  } else {
  MySerial.println("Send failed");
  }
  #endif

  #ifdef BLINK_ON_SEND_SUCCESS
  if (status == ESP_NOW_SEND_SUCCESS) {
    led_status = ~led_status;
    // this function happens too fast to register a blink
    // instead, we latch on/off as data is successfully sent
    digitalWrite(LED_BUILTIN, led_status);
    return;
  }
  // turn off the LED if send fails
  //led_status = 1;
  //digitalWrite(LED_BUILTIN, led_status);
  #endif
}
#endif

void OnDataRecv(const esp_now_recv_info_t * mac_addr, const uint8_t *incomingData, int len) {
  #ifdef BLINK_ON_RECV
  digitalWrite(LED_BUILTIN, LOW);
  #endif
  memcpy(&buf_recv, incomingData, sizeof(buf_recv));
  MySerial.write(buf_recv, len);
  #ifdef BLINK_ON_RECV_SUCCESS
    led_status = ~led_status;
    // this function happens too fast to register a blink
    // instead, we latch on/off as data is successfully recieved
    digitalWrite(LED_BUILTIN, led_status);
   #endif
  #ifdef BLINK_ON_RECV
  digitalWrite(LED_BUILTIN, HIGH);
  #endif
  #ifdef DEBUG
  MySerial.print("\n Bytes received: ");
  MySerial.println(len);
  #endif
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Flash LED 3 times at boot
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(200); // Wait for 200 milliseconds
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200); // Wait for 200 milliseconds
  }

  //Serial.begin(BAUD_RATE, SER_PARAMS, RX_PIN, TX_PIN);
  MySerialSetup;
  MySerial.println("ESP-NOW Boot");
  MySerial.println(send_timeout);
  WiFi.mode(WIFI_STA);

  #ifdef DEBUG
  MySerial.print("ESP32C3 MAC Address: ");
  MySerial.println(WiFi.macAddress());
  #endif
  
  if (esp_wifi_set_channel(WIFI_CHAN, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    #ifdef DEBUG
    MySerial.println("Error changing WiFi channel");
    #endif
    return;
  }

  if (esp_now_init() != ESP_OK) {
    #ifdef DEBUG
    MySerial.println("Error initializing ESP-NOW");
    #endif
    return;
  }

  #if defined(DEBUG) || defined(BLINK_ON_SEND_SUCCESS)
  esp_now_register_send_cb(OnDataSent);
  #endif
  
  // esp_now_peer_info_t peerInfo;  // scope workaround for arduino-esp32 v2.0.1
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = WIFI_CHAN;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    #ifdef DEBUG
    MySerial.println("Failed to add peer");
    #endif
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {

  // read up to BUFFER_SIZE from serial port
  if (MySerial.available()) {
    while (MySerial.available() && buf_size < BUFFER_SIZE) {
      buf_send[buf_size] = MySerial.read();
      send_timeout = micros() + timeout_micros;
      buf_size++;
    }
  }

  // send buffer contents when full or timeout has elapsed
  if (buf_size == BUFFER_SIZE || (buf_size > 0 && micros() >= send_timeout)) {
    #ifdef BLINK_ON_SEND
    digitalWrite(LED_BUILTIN, LOW);
    #endif
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &buf_send, buf_size);
    buf_size = 0;
    #ifdef DEBUG
    if (result == ESP_OK) {
      MySerial.println("Sent!");
    }
    else {
      MySerial.println("Send error");
    }
    #endif
    #ifdef BLINK_ON_SEND
    digitalWrite(LED_BUILTIN, HIGH);
    #endif
  }

}
