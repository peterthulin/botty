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
GPIO.setup(PIN_PWMA, GPIO.OUT) # Connected to PWMA
GPIO.setup(PIN_AIN1, GPIO.OUT) # Connected to AIN1ß
GPIO.setup(PIN_AIN2, GPIO.OUT) # Connected to AIN2

GPIO.setup(PIN_PWMB, GPIO.OUT) # Connected to PWMB
GPIO.setup(PIN_BIN1, GPIO.OUT) # Connected to AIN2
GPIO.setup(PIN_BIN2, GPIO.OUT) # Connected to AIN1

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

# # Set the motor speed
# GPIO.output(7, GPIO.HIGH) # Set PWMA

# # Disable STBY (standby)
# GPIO.output(13, GPIO.HIGH)

# # Wait 5 seconds
# time.sleep(5)

# # Drive the motor counterclockwise
# GPIO.output(12, GPIO.LOW) # Set AIN1
# GPIO.output(11, GPIO.HIGH) # Set AIN2

# # Set the motor speed
# GPIO.output(7, GPIO.HIGH) # Set PWMA

# # Disable STBY (standby)
# GPIO.output(13, GPIO.HIGH)

# # Wait 5 seconds
# time.sleep(5)

# # Reset all the GPIO pins by setting them to LOW
# GPIO.output(12, GPIO.LOW) # Set AIN1
# GPIO.output(11, GPIO.LOW) # Set AIN2
# GPIO.output(7, GPIO.LOW) # Set PWMA
# GPIO.output(13, GPIO.LOW) # Set STBY

