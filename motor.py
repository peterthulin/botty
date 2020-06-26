import time
import RPi.GPIO as GPIO


class MotorDriver_TB6612():

	def __init__(self):
		# TODO: let pins be input parameters
		self.PIN_PWMA = 4
		self.PIN_AIN1 = 18
		self.PIN_AIN2 = 17
		self.PIN_PWMB = 22
		self.PIN_BIN1 = 27
		self.PIN_BIN2 = 23
		self.all_pins = [
			self.PIN_PWMA, self.PIN_AIN1, self.PIN_AIN2,
			self.PIN_PWMB, self.PIN_BIN1, self.PIN_BIN2
		]

	self._init_gpio()

	def _init_gpio(self):
		GPIO.setup(self.PIN_PWMA, GPIO.OUT)
		GPIO.setup(self.PIN_AIN1, GPIO.OUT)
		GPIO.setup(self.PIN_AIN2, GPIO.OUT)
		GPIO.setup(self.PIN_PWMB, GPIO.OUT)
		GPIO.setup(self.PIN_BIN1, GPIO.OUT)
		GPIO.setup(self.PIN_BIN2, GPIO.OUT)

	def forward(self):
		GPIO.output(self.PIN_AIN1, GPIO.HIGH)
		GPIO.output(self.PIN_AIN2, GPIO.LOW)
		GPIO.output(self.PIN_PWMA, GPIO.HIGH)
		GPIO.output(self.PIN_BIN1, GPIO.HIGH)
		GPIO.output(self.PIN_BIN2, GPIO.LOW)
		GPIO.output(self.PIN_PWMB, GPIO.HIGH)

	def backward(self):
		GPIO.output(self.PIN_AIN1, GPIO.LOW)
		GPIO.output(self.PIN_AIN2, GPIO.HIGH)
		GPIO.output(self.PIN_PWMA, GPIO.HIGH)
		GPIO.output(self.PIN_BIN1, GPIO.LOW)
		GPIO.output(self.PIN_BIN2, GPIO.HIGH)
		GPIO.output(self.PIN_PWMB, GPIO.HIGH)

	def rotate_left(self):
		GPIO.output(self.PIN_AIN1, GPIO.LOW)
		GPIO.output(self.PIN_AIN2, GPIO.HIGH)
		GPIO.output(self.PIN_PWMA, GPIO.HIGH)
		GPIO.output(self.PIN_BIN1, GPIO.HIGH)
		GPIO.output(self.PIN_BIN2, GPIO.LOW)
		GPIO.output(self.PIN_PWMB, GPIO.HIGH)

	def rotate_right(self):
		GPIO.output(self.PIN_AIN1, GPIO.HIGH)
		GPIO.output(self.PIN_AIN2, GPIO.LOW)
		GPIO.output(self.PIN_PWMA, GPIO.HIGH)
		GPIO.output(self.PIN_BIN1, GPIO.LOW)
		GPIO.output(self.PIN_BIN2, GPIO.HIGH)
		GPIO.output(self.PIN_PWMB, GPIO.HIGH)

	def stop(self):
		for pin in self.all_pins:
			GPIO.output(pin, GPIO.LOW)


def main():
    """ Simple main loop for testing """
    motor_driver = MotorDriver_TB6612()

    # Test all motor functions for 1 sec
    motor_driver.forward()
    time.sleep(1.0)

    motor_driver.backward()
    time.sleep(1.0)

    motor_driver.rotate_left()
    time.sleep(1.0)

    motor_driver.rotate_right()
    time.sleep(1.0)


if __name__ == '__main__':
    main()
