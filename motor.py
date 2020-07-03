import argparse
import time

import RPi.GPIO as GPIO


class MotorDriver_TB6612():

    def __init__(self, pin_config, a_is_left=True):
        if a_is_left:
            self.left_pwm = pin_config["PWMA"]
            self.left_in1 = pin_config["AIN1"]
            self.left_in2 = pin_config["AIN2"]
            self.right_pwm = pin_config["PWMB"]
            self.right_in1 = pin_config["BIN1"]
            self.right_in2 = pin_config["BIN2"]
        else:
            self.left_pwm = pin_config["PWMB"]
            self.left_in1 = pin_config["BIN1"]
            self.left_in2 = pin_config["BIN2"]
            self.right_pwm = pin_config["PWMA"]
            self.right_in1 = pin_config["AIN1"]
            self.right_in2 = pin_config["AIN2"]

        self.all_pins = [
            self.left_pwm, self.left_in1, self.left_in2,
            self.right_pwm, self.right_in1, self.right_in2
        ]

        self._init_gpio()

    def _init_gpio(self):
        GPIO.setup(self.left_pwm, GPIO.OUT)
        GPIO.setup(self.left_in1, GPIO.OUT)
        GPIO.setup(self.left_in2, GPIO.OUT)
        GPIO.setup(self.right_pwm, GPIO.OUT)
        GPIO.setup(self.right_in1, GPIO.OUT)
        GPIO.setup(self.right_in2, GPIO.OUT)

        self.left_pwm_instance = GPIO.PWM(self.left_pwm, 0.0)
        self.right_pwm_instance = GPIO.PWM(self.right_pwm, 0.0)

    def left_forward(self, dc=100.0, freq=10.0):
        GPIO.output(self.left_in1, GPIO.HIGH)
        GPIO.output(self.left_in2, GPIO.LOW)
        self.set_left_pwm(dc, freq)

    def right_forward(self, dc=100.0, freq=10.0):
        GPIO.output(self.right_in1, GPIO.HIGH)
        GPIO.output(self.right_in2, GPIO.LOW)
        self.set_left_pwm(dc, freq)

    def left_backward(self, dc=100.0, freq=10.0):
        GPIO.output(self.left_in1, GPIO.LOW)
        GPIO.output(self.left_in2, GPIO.HIGH)
        self.set_right_pwm(dc, freq)

    def right_backward(self, dc=100.0, freq=10.0):
        GPIO.output(self.right_in1, GPIO.LOW)
        GPIO.output(self.right_in2, GPIO.HIGH)
        self.set_right_pwm(dc, freq)

    def stop(self):
        for pin in self.all_pins:
            GPIO.output(pin, GPIO.LOW)
        self.left_pwm_instance.ChangeDutyCycle(0.0)
        self.right_pwm_instance.ChangeDutyCycle(0.0)

    def set_left_pwm(self, dc, freq):
        self.left_pwm_instance.ChangeFrequency(freq)
        self.left_pwm_instance.ChangeDutyCycle(dc)

    def set_right_pwm(self, dc, freq):
        self.right_pwm_instance.ChangeFrequency(freq)
        self.right_pwm_instance.ChangeDutyCycle(dc)


def main():
    """ Simple main loop for testing """
    args = parse_args()
    dc = args.pwm_dc
    freq = args.pwm_freq

    pin_config = {
        "PWMA": 12,
        "AIN1": 20,
        "AIN2": 16,
        "PWMB": 13,
        "BIN1": 5,
        "BIN2": 6
    }

    GPIO.setmode(GPIO.BCM)
    motor_driver = MotorDriver_TB6612(pin_config)

    # Test all motor functions for 1 sec
    motor_driver.left_forward(dc, freq)
    time.sleep(1.0)
    motor_driver.stop()

    motor_driver.right_forward(dc, freq)
    time.sleep(1.0)
    motor_driver.stop()

    motor_driver.left_backward(dc, freq)
    time.sleep(1.0)
    motor_driver.stop()

    motor_driver.right_backward(dc, freq)
    time.sleep(1.0)
    motor_driver.stop()


def parse_args():
    parser = argparse.ArgumentParser(description="Test program for HY-SRF05 sensor.")
    parser.add_argument(
        '-d', '--pwm-dc', type=float, default=50.0,
        help='Duty cycle percentage for PWM: [0.0, 100.0]')
    parser.add_argument(
        '-f', '--pwm-freq', type=float, default=10.0,
        help='Frequency of PWM in Hz.')
    return parser.parse_args()


if __name__ == '__main__':
    main()
