#!/bin/bash

# Set GPIO 34 as output and set it to low for buzzer
GPIO_BUZZER=34
echo $GPIO_BUZZER > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio$GPIO_BUZZER/direction
echo 0 > /sys/class/gpio/gpio$GPIO_BUZZER/active_low
echo 0 > /sys/class/gpio/gpio$GPIO_BUZZER/value

# Set GPIO 27 for motor 1 pwm
M1_PWM=27
echo $M1_PWM > /sys/class/gpio/export
echo pwm > /sys/class/gpio/gpio$M1_PWM/direction
echo 0 > /sys/class/gpio/gpio$M1_PWM/active_low
echo 0 > /sys/class/gpio/gpio$M1_PWM/value

# Set GPIO 63 for motor 1 direction
M1_DIR=63
echo $M1_DIR > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio$M1_DIR/direction
echo 0 > /sys/class/gpio/gpio$M1_DIR/active_low
echo 0 > /sys/class/gpio/gpio$M1_DIR/value

# Set GPIO 62 for motor 1 brake
M1_BRAKE=62
echo $M1_BRAKE > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio$M1_BRAKE/direction
echo 0 > /sys/class/gpio/gpio$M1_BRAKE/active_low
echo 1 > /sys/class/gpio/gpio$M1_BRAKE/value

# Set GPIO 26 for motor 1 feedback
M1_FEEDBACK=36
echo $M1_FEEDBACK > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio$M1_FEEDBACK/direction
echo raising > /sys/class/gpio/gpio$M1_FEEDBACK/edge

# Set GPIO 33 for motor 2 pwm
M2_PWM=33
echo $M2_PWM > /sys/class/gpio/export
echo pwm > /sys/class/gpio/gpio$M2_PWM/direction
echo 0 > /sys/class/gpio/gpio$M2_PWM/active_low
echo 0 > /sys/class/gpio/gpio$M2_PWM/value

# Set GPIO 37 for motor 2 direction
M2_DIR=37
echo $M2_DIR > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio$M2_DIR/direction
echo 0 > /sys/class/gpio/gpio$M2_DIR/active_low
echo 0 > /sys/class/gpio/gpio$M2_DIR/value

# Set GPIO 28 for motor 2 brake
M2_BRAKE=28
echo $M2_BRAKE > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio$M2_BRAKE/direction
echo 0 > /sys/class/gpio/gpio$M2_BRAKE/active_low
echo 1 > /sys/class/gpio/gpio$M2_BRAKE/value

# Set GPIO 32 for motor 2 feedback
M2_FEEDBACK=32
echo $M2_FEEDBACK > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio$M2_FEEDBACK/direction
echo raising > /sys/class/gpio/gpio$M2_FEEDBACK/edge

# Set GPIO 36 for D0
D0=36
echo $D0 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio$D0/direction
echo 0 > /sys/class/gpio/gpio$D0/active_low
echo 0 > /sys/class/gpio/gpio$D0/value

# Set GPIO 39 for D1
D1=39
echo $D1 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio$D1/direction
echo 0 > /sys/class/gpio/gpio$D1/active_low
echo 0 > /sys/class/gpio/gpio$D1/value

# Set GPIO 40 for D2
D2=40
echo $D2 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio$D2/direction
echo 1 > /sys/class/gpio/gpio$D2/active_low
echo 0 > /sys/class/gpio/gpio$D2/value

# Set GPIO 41 for D3
D3=41
echo $D3 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio$D3/direction
echo 1 > /sys/class/gpio/gpio$D3/active_low
echo 0 > /sys/class/gpio/gpio$D3/value
