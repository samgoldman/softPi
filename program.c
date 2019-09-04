#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "softPi.h"


#define SWITCH 14
#define MOTOR_1 15    // RIGHT
#define MOTOR_DIR_1 16
#define MOTOR_2 17    // LEFT
#define MOTOR_DIR_2 18

void alert(int gpio, int level, uint32_t tick)
{
    if (PI_ON == level)
    {
        printf("Bump!\n");

        // Backwards one second
        gpioWrite(MOTOR_DIR_1, PI_ON);
        gpioWrite(MOTOR_DIR_2, PI_ON);
        sleep(1);

        // Forwards again, but slower on one side for a second
        gpioWrite(MOTOR_DIR_1, PI_OFF);
        gpioWrite(MOTOR_DIR_2, PI_OFF);
        gpioPWM(MOTOR_1, 50);
        gpioPWM(MOTOR_2, 60);
        sleep(1);

        // Normal again
        gpioPWM(MOTOR_1, 255);
        gpioPWM(MOTOR_2, 255);
    }
}

int main(int argc, char *argv[])
{
    int secs=600;

    if (gpioInitialise()<0) return 1;

    gpioSetMode(SWITCH, PI_INPUT);
    gpioSetMode(MOTOR_DIR_1, PI_OUTPUT);
    gpioSetMode(MOTOR_DIR_2, PI_OUTPUT);

    gpioWrite(MOTOR_DIR_1, PI_OFF);
    gpioWrite(MOTOR_DIR_2, PI_OFF);

    gpioPWM(MOTOR_1, 255);
    gpioPWM(MOTOR_2, 255);

    gpioSetAlertFunc(SWITCH, alert);

    sleep(secs);

    gpioTerminate();
}
