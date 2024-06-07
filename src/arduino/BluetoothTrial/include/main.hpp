#ifndef MAIN_H
#define MAIN_H
#include <Arduino.h>
#include <WString.h>
#include <L298NX2.h>


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#define SERIAL_SIZE_RX 2048 //should fix the overflow error
#define MAX_COMMANDS 10
void processMessage(String message);
void processMessage(String message, L298NX2 firstBoard, L298NX2 secondBoard);

#endif /*MAIN_H*/