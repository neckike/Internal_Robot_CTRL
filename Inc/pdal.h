#ifndef PDAL_H
#define PDAL_H

#define   GPIO_HIGH    1
#define   GPIO_LOW     0

#define   PWM_HIGH     100
#define   PWM_HALF     50
#define   PWM_LOW      0

/*
 * Temporal typedefs
 */
typedef unsigned char Byte;
typedef unsigned char GpioPort;
typedef unsigned char GpioPin;

/*
 * AL functions template
 */
void initPort(GpioPort port) { }
void initGpio(GpioPin port)  { }
void initPwm()  { }

#endif // PDAL_H
