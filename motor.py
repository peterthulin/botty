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

    def left_forward(self):
        GPIO.output(self.left_in1, GPIO.HIGH)
        GPIO.output(self.left_in2, GPIO.LOW)
        GPIO.output(self.left_pwm, GPIO.HIGH)

    def right_forward(self):
        GPIO.output(self.right_in1, GPIO.HIGH)
        GPIO.output(self.right_in2, GPIO.LOW)
        GPIO.output(self.right_pwm, GPIO.HIGH)

    def left_backward(self):
        GPIO.output(self.left_in1, GPIO.LOW)
        GPIO.output(self.left_in2, GPIO.HIGH)
        GPIO.output(self.left_pwm, GPIO.HIGH)

    def right_backward(self):
        GPIO.output(self.right_in1, GPIO.LOW)
        GPIO.output(self.right_in2, GPIO.HIGH)
        GPIO.output(self.right_pwm, GPIO.HIGH)

    def stop(self):
        for pin in self.all_pins:
            GPIO.output(pin, GPIO.LOW)


def main():
    """ Simple main loop for testing """

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
    motor_driver.left_forward()
    time.sleep(1.0)
    motor_driver.stop()

    motor_driver.right_forward()
    time.sleep(1.0)
    motor_driver.stop()

    motor_driver.left_backward()
    time.sleep(1.0)
    motor_driver.stop()

    motor_driver.right_backward()
    time.sleep(1.0)
    motor_driver.stop()


if __name__ == '__main__':
    main()
