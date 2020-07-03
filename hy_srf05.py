"""
hy_srf05.py

Simple class for HY_SRF05 ultrasonic sensor.

Credit to original example by Eric Forler:
https://erich.forler.ca/component/content/article/33-blog-hands-on-tech/146-raspberry-pi-ultrasonic-distance-sensor-hr-srf05-full-lesson
"""

import RPi.GPIO as GPIO
import time
import numpy as np

from collections import namedtuple

# Container class representing an edge on a digital pin.
# Contains the mode of the pin after readout and the time of reading.
TimedPinEdge = namedtuple('TimedPinEdge', ['time', 'mode'])


class HY_SRF05():
    """
    Class for easy use of HY-SRF05 ultrasonic sensor.
    """

    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin

        # Assume speed of sound is 343 m/s. The pulse has to travel the distance twice.
        self.time_to_distance_factor = 343.0 / 2.0

        # Amount of time in seconds that the system sleeps before sending another
        # sample request to the sensor. Sensor has an internal time out of 30 ms
        # so we should not be lower than that.
        self.sample_sleep = 0.03
        self.last_trigger = time.monotonic()

        # Sleep time after trigger call should be ~10 microseconds
        self.trigger_sleep = 0.00001

        # Container for rising and falling edges on echo pin
        self.echo_stack = []

        self.init_gpio_pins()

    def init_gpio_pins(self):
        """
        Configure the pins on the Raspberry Pi and add the echo pin callback.
        """
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.echo_pin, GPIO.BOTH, callback=self.echo_callback)

    def echo_callback(self, channel):
        """
        Callback function when an edge is detected on the echo pin.
        Stores the measurement as a TimedPinEdge.
        """
        now = time.monotonic()
        self.echo_stack.append(TimedPinEdge(now, GPIO.input(self.echo_pin)))

    def trigger(self):
        """
        Generates a 10 microseconds pulse on the trigger pin
        """
        # Make sure we don't trigger too often
        now = time.monotonic()
        if now - self.last_trigger < self.sample_sleep:
            GPIO.output(self.trigger_pin, GPIO.HIGH)
            time.sleep(self.trigger_sleep)
            GPIO.output(self.trigger_pin, GPIO.LOW)
            self.last_trigger = now

    def get_distance(self, send_trigger=True):
        """
        Generate an ultrasonic pulse and calculate the distance using the time difference
        on the rising and falling edge on the echo pin
        """

        # Tell the sensor to send out an ultrasonic pulse.
        if send_trigger:
            self.trigger()

        # Check if there are enough edges on the echo_stack
        if len(self.echo_stack) < 2:
            return None

        # Search for rising to falling edge pairs
        time_diffs = []
        stack_index = 1
        while stack_index < len(self.echo_stack):
            first_edge = self.echo_stack[stack_index - 1]
            second_edge = self.echo_stack[stack_index]
            if first_edge.mode == 1 and second_edge.mode == 0:
                # Found pair of rising to falling edge
                time_diff = second_edge.time - first_edge.time
                time_diffs.append(time_diff)
                # We can skip ahead one step on the stack search
                stack_index += 1
            stack_index += 1

        time_diffs = np.array(time_diffs)
        distances = time_diffs * self.time_to_distance_factor

        self.clear_echo_stack()

        return distances.mean()

    def clear_echo_stack(self):
        """
        Clear the stack but keep the last element if it is a rising edge
        """
        # TODO: do we need a lock for the stack?
        last_edge = self.echo_stack[-1]
        self.echo_stack.clear()
        if last_edge.mode == 1:
            self.echo_stack.append(last_edge)


def main():
    """ Simple main loop for testing """
    trigger_pin = 25
    echo_pin = 24
    sensor = HY_SRF05(trigger_pin, echo_pin)
    while True:
        distance = sensor.get_distance(send_trigger=True)
        if distance:
            print(f"Distance: {distance:1.4f} m")


if __name__ == '__main__':
    main()
