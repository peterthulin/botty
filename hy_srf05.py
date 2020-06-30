"""
hy_srf05.py

Simple class for HY_SRF05 ultrasonic sensor.

Credit to original example by Eric Forler:
https://erich.forler.ca/component/content/article/33-blog-hands-on-tech/146-raspberry-pi-ultrasonic-distance-sensor-hr-srf05-full-lesson
"""

import RPi.GPIO as GPIO
import time

from collections import namedtuple

# Container class representing an edge on a digital pin.
# Contains the mode of the pin after readout and the time of reading.
TimedPinEdge = namedtuple('TimedPinEdge', ['time', 'mode'])


class HY_SRF05():

    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin

        # Assume speed of sound is 343 m/s. The pulse has to travel the distance twice.
        self.time_to_distance_factor = 343.0 / 2.0

        # amount of time in seconds that the system sleeps before sending another
        # sample request to the sensor
        self.sample_sleep = 0.01

        # Sleep time after trigger call
        self.trigger_sleep = 0.00001

        # Time out in seconds in case the program gets stuck in a loop
        # 0.05 seconds means we can measure at most 340 * 0.05 / 2 = 8.5 meters
        self.time_out = .05

        # Container for rising and falling edges on echo pin
        self.echo_stack = []

        self.init_gpio_pins()

    def init_gpio_pins(self):
        # TODO: where to set the GPIO pin mode?
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.echo_pin, GPIO.BOTH, callback=self.echo_callback)

    def echo_callback(self, channel):
        """
        Callback function when the rising edge is detected on the echo pin.
        Stores a tuple where the first element is the time and the second is
        the mode of the pin on readout.
        """
        now = time.monotonic()
        self.echo_stack.append(TimedPinEdge(now, GPIO.input(self.echo_pin)))

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

        self.echo_stack.clear()

        # Tell the sensor to send out an ultrasonic pulse.
        self.trigger()
        start = time.monotonic()

        # Wait for the echo pulse while checking the timeout condition
        while len(self.echo_stack) < 2 and (time.monotonic() < start + self.time_out):
            pass

        # Calculate the distance based on the timing difference of the echo pin edges
        distance = None
        if len(self.echo_stack) == 2:
            first_edge = self.echo_stack[0]
            second_edge = self.echo_stack[1]
            # Make sure we have captured a rising and falling edge on the echo pin
            if first_edge.mode == 1 and second_edge.mode == 0:
                time_diff = second_edge.time - first_edge.time
                distance = time_diff * self.time_to_distance_factor
            else:
                print("Conflicting pin edges")

        # Pause to make sure we don't overload the sensor with requests
        time.sleep(self.sample_sleep)

        return distance


def main():
    """ Simple main loop for testing """
    trigger_pin = 25
    echo_pin = 24
    sensor = HY_SRF05(trigger_pin, echo_pin)
    while True:
        sensor.trigger()
        time.wait(0.1)
        print(sensor.echo_stack)
        print(f"Diff: {sensor.echo_stack[1] - sensor.echo_stack[0]}")
        sensor.echo_stack.clear()


if __name__ == '__main__':
    main()
