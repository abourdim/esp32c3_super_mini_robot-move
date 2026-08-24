#ifndef __DEF_INCLUDE_SERVOS_H__
#define __DEF_INCLUDE_SERVOS_H__

extern void moveServos(int32_t leftSpeed, int32_t rightSpeed);
extern void stopServos();

// Sonar head. moveHead() clamps to the mechanical limits in 00_config.h;
// headAngle() reports where it actually went, which is what the radar
// widget is plotted against -- not what was asked for.
extern void moveHead(int32_t angle);
extern void centerHead();
extern uint8_t headAngle();

#endif // __DEF_INCLUDE_SERVOS_H__