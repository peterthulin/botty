import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

# set up GPIO pins
GPIO.setup(7, GPIO.OUT) # Connected to PWMA
GPIO.setup(11, GPIO.OUT) # Connected to AIN2
GPIO.setup(12, GPIO.OUT) # Connected to AIN1
GPIO.setup(13, GPIO.OUT) # Connected to PWMA
GPIO.setup(16, GPIO.OUT) # Connected to AIN2
GPIO.setup(15, GPIO.OUT) # Connected to AIN1

# Drive the motor clockwise
GPIO.output(12, GPIO.HIGH) # Set AIN1
GPIO.output(11, GPIO.LOW) # Set AIN2
GPIO.output(7, GPIO.HIGH) # PWMA

GPIO.output(13, GPIO.HIGH) # Set BIN1
GPIO.output(16, GPIO.LOW)  # Set BIN2
GPIO.output(15, GPIO.HIGH)  # PWMB

time.sleep(1)

GPIO.output(12, GPIO.LOW) # Set AIN1
GPIO.output(11, GPIO.LOW) # Set AIN2
GPIO.output(7, GPIO.LOW)

GPIO.output(13, GPIO.LOW)  # Set BIN1
GPIO.output(16, GPIO.LOW)  # Set BIN2
GPIO.output(15, GPIO.LOW)  # PWMB

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

