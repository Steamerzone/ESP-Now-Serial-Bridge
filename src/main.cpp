/*********************************************************************************
 * ESP-Now-Serial-Bridge (ESP32-C3 EWK 06-08-2025) - SLEEP MODE FIXED
 *
 * ESP32 based serial bridge for transmitting serial data between two boards
 *
 * FIXED ISSUES:
 * - Boot sleep: Now properly initializes activity timer AFTER WiFi setup
 * - Millis rollover: Uses proper unsigned arithmetic for safe rollover handling
 * - Premature sleep: Better activity tracking during serial communication
 * - Debug output: Enhanced debugging for sleep timeout analysis
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

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// Board configuration - manually uncomment #define BOARD1 or #define BOARD2 below
// (or use build flags in platformio.ini: build_flags = -DBOARD1 or -DBOARD2)

#define BOARD1 // BOARD1 or BOARD2

#ifdef BOARD1
//#define RECVR_MAC {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}  // broadcast mode
#define RECVR_MAC {0x64, 0xE8, 0x33, 0x88, 0x26, 0xF8}  // replace with BOARD2 mac address
#define MySerial Serial
#define MySerialSetup MySerial.begin(1000000)            // Use USB serial
//#define BLINK_ON_SEND
#define BLINK_ON_SEND_SUCCESS
#define BLINK_ON_RECV_SUCCESS
//#define BLINK_ON_RECV
// Sleep mode configuration (in seconds, 0 = disabled)
#define SLEEP_TIMEOUT_SECONDS 600 // enter 'sleep mode' (disable wifi) after X seconds of no serial activity
                                  // from master (BOARD1), only serial data from host can wake it

#else

#define RECVR_MAC {0x94, 0xA9, 0x90, 0x47, 0x45, 0x00}  // replace with BOARD1 mac address
HardwareSerial MySerial(1);                             // Use UART1
#define TX_PIN     5                                    // GPIO20 and GPIO21 are reserved for internal RF functions. External connections to them 
#define RX_PIN     6                                    // can degrade Wi-Fi performance.
#define MySerialSetup MySerial.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN)
//#define BLINK_ON_SEND
#define BLINK_ON_SEND_SUCCESS
#define BLINK_ON_RECV_SUCCESS
//#define BLINK_ON_RECV
#define SLEEP_TIMEOUT_SECONDS 0   // Sleep mode disabled for BOARD2 (slave)
#endif

#define WIFI_CHAN  14 // 12-13 only legal in US in lower power mode, do not use 14

#define BUFFER_SIZE 250 // max of 250 bytes
//#define DEBUG // for additional serial messages (may interfere with other messages) - ENABLE FOR DEBUGGING
#define BAUD_RATE 1000000// for calulating timeout
#define LED_BUILTIN 8
#define LED_ON  LOW     // ESP32-C3 pin 8 LED is typically active-low
#define LED_OFF HIGH    // ESP32-C3 pin 8 LED is typically active-low

const uint8_t broadcastAddress[] = RECVR_MAC;
// wait for double the time between bytes at this serial baud rate before sending a packet
// this *should* allow for complete packet forming when using packetized serial comms
const uint32_t timeout_micros = (int)(1.0 / BAUD_RATE * 1E6) * 20;

uint8_t buf_recv[BUFFER_SIZE];
uint8_t buf_send[BUFFER_SIZE];
uint8_t buf_size = 0;
uint32_t send_timeout = 0;
uint8_t led_status = 0;

// Sleep mode variables
#if SLEEP_TIMEOUT_SECONDS > 0
uint32_t last_activity_time = 0;
bool wifi_enabled = true;
bool sleep_timer_initialized = false; // NEW: Track if sleep timer is properly initialized
const uint32_t sleep_timeout_millis = (uint32_t)SLEEP_TIMEOUT_SECONDS * 1000UL; // Use explicit cast and UL
#endif

esp_now_peer_info_t peerInfo;  // scope workaround for arduino-esp32 v2.0.1

// Function declarations
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void update_activity_time(); // Helper function to update activity timestamp

// Helper function to update activity time (only when sleep mode is enabled)
void update_activity_time() {
  #if SLEEP_TIMEOUT_SECONDS > 0
  uint32_t current_time = millis();
  last_activity_time = current_time;
  sleep_timer_initialized = true; // Mark timer as properly initialized
  
  #ifdef DEBUG
  MySerial.print("Activity updated at: ");
  MySerial.print(current_time);
  MySerial.print("ms (timeout in ");
  MySerial.print(sleep_timeout_millis);
  MySerial.println("ms)");
  #endif
  #endif
}

// Function to check if sleep timeout has been reached (handles millis rollover safely)
bool is_sleep_timeout_reached() {
  #if SLEEP_TIMEOUT_SECONDS > 0
  if (!sleep_timer_initialized || !wifi_enabled) {
    return false; // Don't sleep if timer not initialized or WiFi already disabled
  }
  
  uint32_t current_time = millis();
  
  // Handle the case where current_time might be slightly less than last_activity_time
  // due to timing variations or system adjustments
  if (current_time < last_activity_time) {
    // If current time is less than last activity time, it's likely a timing glitch
    // Update the activity time to current time and don't sleep
    #ifdef DEBUG
    MySerial.print("Timing glitch detected - Current: ");
    MySerial.print(current_time);
    MySerial.print("ms < Last activity: ");
    MySerial.print(last_activity_time);
    MySerial.println("ms, adjusting activity time");
    #endif
    last_activity_time = current_time; // Adjust to prevent underflow
    return false;
  }
  
  uint32_t elapsed_time = current_time - last_activity_time; // Now safe from underflow
  
  #ifdef DEBUG
  // Only show debug output when we're very close to timeout (last 30 seconds) or about to sleep
  static uint32_t last_debug_time = 0;
  if (elapsed_time >= sleep_timeout_millis || 
      (elapsed_time > (sleep_timeout_millis - 30000) && (current_time - last_debug_time) > 10000)) {
    
    MySerial.print("Sleep check - Elapsed: ");
    MySerial.print(elapsed_time / 1000);
    MySerial.print("s / ");
    MySerial.print(sleep_timeout_millis / 1000);
    MySerial.print("s");
    if (elapsed_time >= sleep_timeout_millis) {
      MySerial.print(" -> SLEEPING!");
    }
    MySerial.println("");
    last_debug_time = current_time;
  }
  #endif
  
  return elapsed_time >= sleep_timeout_millis;
  #else
  return false; // Sleep disabled
  #endif
}

// Function to initialize ESP-NOW
bool init_espnow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_wifi_set_channel(WIFI_CHAN, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    #ifdef DEBUG
    MySerial.println("Error changing WiFi channel");
    #endif
    return false;
  }

  if (esp_now_init() != ESP_OK) {
    #ifdef DEBUG
    MySerial.println("Error initializing ESP-NOW");
    #endif
    return false;
  }

  #if defined(DEBUG) || defined(BLINK_ON_SEND_SUCCESS)
  esp_now_register_send_cb(OnDataSent);
  #endif
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = WIFI_CHAN;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    #ifdef DEBUG
    MySerial.println("Failed to add peer");
    #endif
    return false;
  }

  esp_now_register_recv_cb(OnDataRecv);
  
  #ifdef DEBUG
  MySerial.println("ESP-NOW initialized successfully");
  #endif
  
  return true;
}

// Function to disable WiFi for power saving
void disable_wifi() {
  #if SLEEP_TIMEOUT_SECONDS > 0
  if (wifi_enabled) {
    #ifdef DEBUG
    MySerial.println("=== ENTERING SLEEP MODE ===");
    MySerial.print("Last activity was ");
    MySerial.print(millis() - last_activity_time);
    MySerial.println("ms ago");
    #endif
    
    esp_now_deinit();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    wifi_enabled = false;
    sleep_timer_initialized = false; // Reset timer initialization flag
    
    // Flash LED pattern to indicate sleep mode
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_BUILTIN, LED_ON);
      delay(100);
      digitalWrite(LED_BUILTIN, LED_OFF);
      delay(100);
    }
    
    #ifdef DEBUG
    MySerial.println("WiFi disabled - now in sleep mode");
    MySerial.flush(); // Ensure debug message is sent before continuing
    #endif
  }
  #endif
}

// Function to re-enable WiFi
void enable_wifi() {
  #if SLEEP_TIMEOUT_SECONDS > 0
  if (!wifi_enabled) {
    #ifdef DEBUG
    MySerial.println("=== WAKING FROM SLEEP MODE ===");
    MySerial.println("Serial activity detected, re-enabling WiFi...");
    #endif
    
    // Flash LED pattern to indicate wake up
    for (int i = 0; i < 2; i++) {
      digitalWrite(LED_BUILTIN, LED_ON);
      delay(250);
      digitalWrite(LED_BUILTIN, LED_OFF);
      delay(250);
    }
    
    if (init_espnow()) {
      wifi_enabled = true;
      // IMPORTANT: Set activity time AFTER WiFi is fully initialized
      update_activity_time();
      #ifdef DEBUG
      MySerial.println("WiFi re-enabled successfully");
      #endif
    } else {
      #ifdef DEBUG
      MySerial.println("Failed to re-enable WiFi");
      #endif
    }
  } else {
    // WiFi already enabled, just update activity time
    update_activity_time();
  }
  #endif
}

#if defined(DEBUG) || defined(BLINK_ON_SEND_SUCCESS)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Update activity time when data is sent (successful or not)
  update_activity_time();

  #ifdef DEBUG
  if (status == ESP_NOW_SEND_SUCCESS) {
    MySerial.println("ESP-NOW: Send success");
  } else {
    MySerial.println("ESP-NOW: Send failed");
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

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  #ifdef BLINK_ON_RECV
  digitalWrite(LED_BUILTIN, LOW);
  #endif
  
  memcpy(&buf_recv, incomingData, sizeof(buf_recv));
  MySerial.write(buf_recv, len);
  
  // Update activity time when data is received
  update_activity_time();
  
  #ifdef BLINK_ON_RECV_SUCCESS
    led_status = ~led_status;
    // this function happens too fast to register a blink
    // instead, we latch on/off as data is successfully received
    digitalWrite(LED_BUILTIN, led_status);
   #endif
  #ifdef BLINK_ON_RECV
  digitalWrite(LED_BUILTIN, HIGH);
  #endif
  #ifdef DEBUG
  MySerial.print("ESP-NOW: Received ");
  MySerial.print(len);
  MySerial.println(" bytes");
  #endif
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_OFF); // Ensure LED starts OFF

  // Flash LED 3 times at boot
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, LED_ON);   // Turn LED ON
    delay(200); // Wait for 200 milliseconds
    digitalWrite(LED_BUILTIN, LED_OFF);  // Turn LED OFF
    delay(200); // Wait for 200 milliseconds
  }

  MySerialSetup;
  
  // Wait a moment for serial to be ready
  delay(100);
  
  MySerial.println("");
  MySerial.println("=== ESP-NOW Serial Bridge Boot ===");
  
  #if SLEEP_TIMEOUT_SECONDS > 0
  MySerial.print("Sleep mode: ENABLED (");
  MySerial.print(SLEEP_TIMEOUT_SECONDS);
  MySerial.println(" second timeout)");
  #else
  MySerial.println("Sleep mode: DISABLED");
  #endif
  
  #ifdef BOARD1
  MySerial.println("Board: BOARD1 (Master/USB)");
  #else
  MySerial.println("Board: BOARD2 (Slave/UART)");
  #endif
  
  #ifdef DEBUG
  MySerial.print("ESP32C3 MAC Address: ");
  MySerial.println(WiFi.macAddress());
  MySerial.print("Target MAC Address: ");
  for (int i = 0; i < 6; i++) {
    MySerial.printf("%02X", broadcastAddress[i]);
    if (i < 5) MySerial.print(":");
  }
  MySerial.println("");
  #endif
  
  MySerial.println("Initializing ESP-NOW...");
  
  // Initialize ESP-NOW
  if (!init_espnow()) {
    MySerial.println("FATAL: Failed to initialize ESP-NOW");
    while(1) {
      digitalWrite(LED_BUILTIN, LED_ON);
      delay(100);
      digitalWrite(LED_BUILTIN, LED_OFF);
      delay(100);
    }
  }

  MySerial.println("ESP-NOW initialization complete");
  
  // IMPORTANT: Initialize activity timer AFTER successful WiFi/ESP-NOW setup
  // This prevents immediate sleep on boot
  #if SLEEP_TIMEOUT_SECONDS > 0
  delay(500); // Give some time for system to stabilize
  update_activity_time(); // This sets sleep_timer_initialized = true
  MySerial.print("Sleep timer initialized at: ");
  MySerial.print(last_activity_time);
  MySerial.println("ms");
  #endif
  
  MySerial.println("=== System Ready ===");
  MySerial.flush(); // Ensure all boot messages are sent
}

void loop() {
  // Check for sleep timeout - ONLY if sleep mode is enabled for this board
  if (is_sleep_timeout_reached()) {
    disable_wifi();
  }

  // read up to BUFFER_SIZE from serial port
  if (MySerial.available()) {
    #ifdef DEBUG
    static uint32_t last_serial_debug = 0;
    if (millis() - last_serial_debug > 5000) { // Debug every 5 seconds during serial activity
      MySerial.print("Serial data available, bytes: ");
      MySerial.println(MySerial.available());
      last_serial_debug = millis();
    }
    #endif
    
    // Update activity time when serial data is available
    update_activity_time();
    
    // If WiFi is disabled and we have serial data, re-enable it
    #if SLEEP_TIMEOUT_SECONDS > 0
    if (!wifi_enabled) {
      enable_wifi();
      // Wait a bit for WiFi to fully initialize before proceeding
      delay(100);
    }
    #endif
    
    while (MySerial.available() && buf_size < BUFFER_SIZE) {
      buf_send[buf_size] = MySerial.read();
      send_timeout = micros() + timeout_micros;
      buf_size++;
      
      // Update activity time periodically during continuous reading
      // to prevent sleep during long data streams
      if ((buf_size % 100) == 0) { // Every 100 bytes
        update_activity_time();
      }
    }
  }

  // send buffer contents when full or timeout has elapsed
  // Only send if WiFi is enabled (or if sleep mode is disabled)
  #if SLEEP_TIMEOUT_SECONDS > 0
  bool should_send = wifi_enabled && (buf_size == BUFFER_SIZE || (buf_size > 0 && micros() >= send_timeout));
  #else
  bool should_send = (buf_size == BUFFER_SIZE || (buf_size > 0 && micros() >= send_timeout));
  #endif

  if (should_send) {
    #ifdef BLINK_ON_SEND
    digitalWrite(LED_BUILTIN, LOW);
    #endif
    
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &buf_send, buf_size);
    
    #ifdef DEBUG
    MySerial.print("Sent ");
    MySerial.print(buf_size);
    MySerial.print(" bytes: ");
    if (result == ESP_OK) {
      MySerial.println("OK");
    } else {
      MySerial.print("ERROR ");
      MySerial.println(result);
    }
    #endif
    
    // Update activity time when sending data (before clearing buffer)
    update_activity_time();
    
    buf_size = 0;
    
    #ifdef BLINK_ON_SEND
    digitalWrite(LED_BUILTIN, HIGH);
    #endif
  }
}