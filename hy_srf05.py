"""
hy_srf05.py

Simple class for HY_SRF05 ultrasonic sensor.

Credit to original example by Eric Forler:
https://erich.forler.ca/component/content/article/33-blog-hands-on-tech/146-raspberry-pi-ultrasonic-distance-sensor-hr-srf05-full-lesson
"""

import RPi.GPIO as GPIO
import time
import statistics


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
        # The number of times the sensor tests the distance and then picks the middle value to return
        self.number_of_samples = 5
        # amount of time in seconds that the system sleeps before sending another sample request to the sensor
        # 0.01 seconds means we can measure at most 340 * 0.01 / 2 = 1.7 meters
        self.sample_sleep = 0.01

        # Sleep time after trigger call
        self.trigger_sleep = 0.00001

        # Time out in seconds in case the program gets stuck in a loop
        self.time_out = .05

        self.samples_list = []
        self.stack = []

        self.init_gpio_pins()

    def init_gpio_pins(self):
        # TODO: where to set the GPIO pin mode?
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.echo_pin, GPIO.BOTH, callback=self.timer_callback)

    def timer_callback(self, channel):
        """
        Callback function when the rising edge is detected on the echo pin
        """
        now = time.monotonic()
        # stores the start and end times for the distance measurement in a LIFO stack
        self.stack.append(now)

    def trigger(self):
        # set our trigger high, triggering a pulse to be sent - a 1/100,000 of a second pulse or 10 microseconds
        GPIO.output(self.trigger_pin, GPIO.HIGH)
        time.sleep(self.trigger_sleep)
        GPIO.output(self.trigger_pin, GPIO.LOW)

    def get_distance(self):
        """
        Generates an ultrasonic pulse and uses the times that are recorded on the stack to calculate the distance
        """
        # Empty the samples list
        self.samples_list.clear()

        # Checks if the samples_list contains the required number_of_samples
        while len(self.samples_list) < self.number_of_samples:
            # Tell the sensor to send out an ultrasonic pulse.
            self.trigger()

            # check the length of stack to see if it contains a start and end time . Wait until 2 items in the list
            while len(self.stack) < 2:
                # waiting for the stack to fill with a start and end time
                # get the time that we enter this loop to track for timeout
                start = time.monotonic()

                # check the timeout condition
                while time.monotonic() < start + self.time_out:
                    pass

                # The system timed out waiting for the echo to come back. Send a new pulse.
                self.trigger()

            # Stack has two elements on it.
            if len(self.stack) == 2:
                # once the stack has two elements in it, store the difference in the samples_list
                self.samples_list.append(self.stack.pop() - self.stack.pop())

            # Somehow we got three items on the stack, so clear the stack
            elif len(self.stack) > 2:
                self.stack.clear()

            # Pause to make sure we don't overload the sensor with requests and allow the noise to die down
            time.sleep(self.sample_sleep)

        # returns the media distance calculation
        return (statistics.median(self.samples_list) * self.calib_ratio)


def main():
    """ Simple main loop for testing """
    trigger_pin = 25
    echo_pin = 24
    sensor = HY_SRF05(trigger_pin, echo_pin)
    while True:
        print(round(sensor.get_distance(), 1))


if __name__ == '__main__':
    main()
