# ESP-Now-Serial-Bridge

ESP32 based serial bridge for transmitting serial data between two boards

The primary purpose of this sketch was to enable a MAVLink serial connection, which I successfully tested at 57600 bps.  In theory, much faster buad rates should work fine, but I have not tested faster than 115200.
 
Range is easily better than regular WiFi, however an external antenna may be required for truly long range messaging, to combat obstacles/walls, and/or to achieve success in an area saturated with 2.4GHz traffic.

To find the MAC address of each board, uncomment the `#define DEBUG` line, and monitor serial output on boot.  Set the OPPOSITE board's IP address as the value for RECVR_MAC in the macros at the top of the sketch.

When uploading the sketch, be sure to define `BOARD1` or `BOARD2` as appropriate before compiling.

-- Yuri - Sep 2021

Based this example - https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/

### License

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.


The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

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
**********************************************************************************

ESP32-C3 ESP-NOW WiFi unicast usb <> win ser <> usb

Final Latency Stats (ms):

  Len      Avg      Min      Max    Samples  Timeouts
    1     1.52     0.09     6.54         45         0
    2     1.76     1.07    11.95         45         0
    3     1.83     1.13     2.94         45         0
    4     1.96     1.16     3.01         45         0
    5     2.07     1.16     3.95         45         0
    6     2.15     1.43     5.37         45         0
    7     2.42     1.61     5.47         45         0
    8     2.47     1.45     3.94         45         0
    9     2.65     1.54     6.41         45         0
   10     2.71     1.61     5.48         45         0
   11     2.84      1.8     5.75         45         0
   12     2.95     1.64     5.99         45         0
   13     2.95      1.9     4.35         45         0
   14     3.17     1.77     5.28         45         0
   15     3.22     1.88     5.94         45         0
   16     3.42     1.78     6.23         45         0
   17     3.66     1.84     7.48         45         0
   18     3.45     1.97     5.55         45         0
   19     3.79        2     5.51         45         0
   20        4     2.38     6.67         45         0
*/
