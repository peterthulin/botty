import argparse
import time

import RPi.GPIO as GPIO

import hy_srf05
import motor

DESCRIPTION = \
    """
    Very simple robot program that avoids obstacles based on HY-SRF05 ultrasonic
    sensor. The program outline is essentially the following:
    - Check distance
    - If distance > threshold then go forwarde
    - Else rotate left/right
    """

# HY-SRF05 pins
# TODO: make this more configurable
TRIGGER_PIN = 25
ECHO_PIN = 24


def main():
    args = parse_args()

    avoid_obstacles(args.threshold, args.timeout)


def parse_args():
    parser = argparse.ArgumentParser(description=DESCRIPTION)
    parser.add_argument(
        '-d', '--distance-threshold', type=float, default=20.0,
        help='Distance threshold in cm to start rotating.')
    parser.add_argument(
        '-t', '--timeout', type=float, default=60.0,
        help='Timeout in seconds. Halt program after this time.')
    return parser.parse_args()


def avoid_obstacles(threshold, timeout):
    GPIO.setmode(GPIO.BCM)

    motor_driver = motor.MotorDriver_TB6612()
    distance_sensor = hy_srf05(
        trigger_pin=TRIGGER_PIN,
        echo_pin=ECHO_PIN)

    # Wrap loop in try-catch to be able to halt robot before program exit
    try:
        _avoid_obstacles_loop(motor_driver, distance_sensor, threshold, timeout)
    except KeyboardInterrupt:
        pass

    motor_driver.stop()


def _avoid_obstacles_loop(motor_driver, distance_sensor, threshold, timeout):
    start_time = time.time()
    while time.time() - start_time < timeout:
        distance = distance_sensor.get_distance()
        if distance > threshold:
            motor_driver.forward()
        else:
            motor_driver.rotate_left()


if __name__ == '__main__':
    main()
