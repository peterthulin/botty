import time
import RPi.GPIO as GPIO

PIN_PWMA = 7
PIN_AIN1 = 12
PIN_AIN2 = 11

PIN_PWMB = 13
PIN_BIN1 = 16
PIN_BIN2 = 15

ALL_PINS = [
	PIN_PWMA, PIN_AIN1, PIN_AIN2,
	PIN_PWMB, PIN_BIN1, PIN_BIN2]

GPIO.setmode(GPIO.BOARD)
# GPIO.setmode(GPIO.BCM)

# set up GPIO pins
GPIO.setup(PIN_PWMA, GPIO.OUT)
GPIO.setup(PIN_AIN1, GPIO.OUT)
GPIO.setup(PIN_AIN2, GPIO.OUT)

GPIO.setup(PIN_PWMB, GPIO.OUT)
GPIO.setup(PIN_BIN1, GPIO.OUT)
GPIO.setup(PIN_BIN2, GPIO.OUT)

# Drive the motor clockwise
GPIO.output(PIN_AIN1, GPIO.HIGH)
GPIO.output(PIN_AIN2, GPIO.LOW)
GPIO.output(PIN_PWMA, GPIO.HIGH)

GPIO.output(PIN_BIN1, GPIO.HIGH)
GPIO.output(PIN_BIN2, GPIO.LOW)
GPIO.output(PIN_PWMB, GPIO.HIGH)

time.sleep(1)

# Set all pins to low
for pin in ALL_PINS:
	GPIO.output(pin, GPIO.LOW)
