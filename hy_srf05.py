"""
hy_srf05.py

Simple class for HY_SRF05 ultrasonic sensor.

Credit to original example by Eric Forler:
https://erich.forler.ca/component/content/article/33-blog-hands-on-tech/146-raspberry-pi-ultrasonic-distance-sensor-hr-srf05-full-lesson
"""

import RPi.GPIO as GPIO
import time


class HY_SRF05():

    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin

        # Configure calibration
        # TODO: improve this, e.g. with multiple calibration distances
        calibration1 = 30     # the distance the sensor was calibrated at
        calibration2 = 1750   # the median value reported back from the sensor at 30 cm
        calib_ratio = 1000000 * calibration1 / calibration2
        self.calib_ratio = calib_ratio

        # TODO: make these inputs
        # The number of times the sensor tests the distance and
        # then picks the middle value to return
        self.number_of_samples = 5
        # amount of time in seconds that the system sleeps before sending another
        # sample request to the sensor
        self.sample_sleep = 0.01

        # Sleep time after trigger call
        self.trigger_sleep = 0.00001

        # Time out in seconds in case the program gets stuck in a loop
        # 0.05 seconds means we can measure at most 340 * 0.05 / 2 = 8.5 meters
        self.time_out = .05

        self.stack = []

        self.init_gpio_pins()

    def init_gpio_pins(self):
        # TODO: where to set the GPIO pin mode?
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.echo_pin, GPIO.BOTH, callback=self.trigger_callback)

    def trigger_callback(self, channel):
        """
        Callback function when the rising edge is detected on the echo pin
        """
        now = time.monotonic()
        # stores the start and end times for the distance measurement in a LIFO stack
        if GPIO.input(self.trigger_pin) == 0:
            print(f"Falling: {now}")
        else:
            print(f"Rising: {now}")
        self.stack.append(now)

    def trigger(self):
        """
        Create a 10 microseconds pulse on the trigger pin
        """
        GPIO.output(self.trigger_pin, GPIO.HIGH)
        time.sleep(self.trigger_sleep)
        GPIO.output(self.trigger_pin, GPIO.LOW)

    def get_distance(self):
        """
        Generate an ultrasonic pulse and calculate the distance using the time difference
        on the rising and falling edge on the echo pin
        """

        self.stack.clear()

        # Tell the sensor to send out an ultrasonic pulse.
        self.trigger()
        start = time.monotonic()

        # Wait for the echo pulse while checking the timeout condition
        while len(self.stack) < 2 and (time.monotonic() < start + self.time_out):
            pass

        # If we have two elements on the stack we can calculate the distance
        distance = None
        if len(self.stack) == 2:
            time_diff = self.stack.pop() - self.stack.pop()
            distance = time_diff * self.calib_ratio
        else:
            print(f"Stack size: {len(self.stack)}")

        # Pause to make sure we don't overload the sensor with requests
        time.sleep(self.sample_sleep)

        return distance


def main():
    """ Simple main loop for testing """
    trigger_pin = 25
    echo_pin = 24
    sensor = HY_SRF05(trigger_pin, echo_pin)
    while True:
        print(sensor.get_distance())


if __name__ == '__main__':
    main()
