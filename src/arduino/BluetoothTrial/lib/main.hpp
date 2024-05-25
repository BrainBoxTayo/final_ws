#ifndef MAIN_H
#define MAIN_H
#include <Arduino.h>
#include <WString.h>


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define MAX_COMMANDS 4

int *tokenizetoint(String input);

#endif /*MAIN_H*/