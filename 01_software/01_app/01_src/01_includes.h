#ifndef __DEF_INCLUDE_INCLUDES_H__
#define __DEF_INCLUDE_INCLUDES_H__

#include <Arduino.h>  // Essential for Arduino functions
#include <ESP32Servo.h>


#include "00_config.h"
// #include "01_includes.h"
#include "02_data.h"
#include "03_bit-rxy.h"
#include "04_tasks.h"
#include "08_button.h"
#include "09_servos.h"
#include "11_events.h"
#include "12_ultrasonic.h"
#include "17_ota.h"

uint8_t remotexy_get_connect_flag( );

#endif // __DEF_INCLUDE_INCLUDES_H__