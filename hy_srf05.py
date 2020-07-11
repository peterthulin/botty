"""
hy_srf05.py

Simple class for HY_SRF05 ultrasonic sensor.

Credit to original example by Eric Forler:
https://erich.forler.ca/component/content/article/33-blog-hands-on-tech/146-raspberry-pi-ultrasonic-distance-sensor-hr-srf05-full-lesson
"""

import argparse
import time
import threading
from collections import namedtuple

import numpy as np
import RPi.GPIO as GPIO

# Container class representing an edge on a digital pin.
# Contains the mode of the pin after readout and the time of reading.
TimedPinEdge = namedtuple('TimedPinEdge', ['time', 'mode'])


class HY_SRF05():
    """
    Class for easy use of HY-SRF05 ultrasonic sensor.
    """

    def __init__(self,
                 trigger_pin,
                 echo_pin,
                 trigger_sleep=0.05,
                 start_trigger_thread=False,
                 accept_inverted_pairs=True):

        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.accept_inverted_pairs = accept_inverted_pairs

        # Assume speed of sound is 343 m/s. The pulse has to travel the distance twice.
        self.time_to_distance_factor = 343.0 / 2.0

        # Amount of time required before allowing a new trigger to be sent.
        # Sensor has an internal time out of 30 ms so we should not be lower than that.
        if trigger_sleep < 0.03:
            trigger_sleep = 0.03
        self.trigger_sleep = trigger_sleep
        self.last_trigger = 0.0

        # Sleep time after trigger call should be ~10 microseconds
        self.trigger_pulse_sleep = 0.00001

        # Container for rising and falling edges on echo pin
        self.echo_stack = []

        self.init_gpio_pins()

        if start_trigger_thread:
            # Create daemon thread to automatically exit on main program exit
            self.trigger_thread = threading.Thread(
                target=self.auto_trigger_loop, name='trigger_thread', daemon=True)
            self.trigger_thread.start()
        else:
            self.trigger_thread = None

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
        Generates a 10 microseconds pulse on the trigger pin if elapsed time
        since last trigger is more than the specified trigger sleep.
        """
        # Make sure we don't trigger too often
        now = time.monotonic()
        if now - self.last_trigger > self.trigger_sleep:
            GPIO.output(self.trigger_pin, GPIO.HIGH)
            time.sleep(self.trigger_pulse_sleep)
            GPIO.output(self.trigger_pin, GPIO.LOW)
            self.last_trigger = now

    def auto_trigger_loop(self):
        """
        Create a loop that automatically triggers pulses on the sensor.
        """
        while True:
            self.trigger()
            time.sleep(self.trigger_sleep)

    def get_distance(self, send_trigger=True):
        """
        Generate an ultrasonic pulse and calculate the distance using the time difference
        on the rising and falling edge on the echo pin
        """

        # Tell the sensor to send out an ultrasonic pulse.
        if send_trigger:
            self.trigger()

        # Process the edges on the stack and get the time differences
        time_diffs = self._process_echo_stack()

        distance = None

        if len(time_diffs) > 0:
            distances = time_diffs * self.time_to_distance_factor
            distance = distances.mean()

        return distance

    def _process_echo_stack(self):
        """
        Go through the echo_stack while clearing it. Some of the pin edge variants
        should technically be impossible but can happen due to noise on the pins.
        Therefore we have to handle all cases. Only when we have a rising to falling
        edge do we treat it as a true sample and calculate the time difference.
        """
        time_diffs = []
        while len(self.echo_stack) > 1:
            first_edge = self.echo_stack[0]
            second_edge = self.echo_stack[1]
            if first_edge.mode == 1 and second_edge.mode == 0:
                # Found pair of rising to falling edge
                time_diff = second_edge.time - first_edge.time
                time_diffs.append(time_diff)
                self.echo_stack.remove(first_edge)
                self.echo_stack.remove(second_edge)
            elif first_edge.mode == 0 and second_edge.mode == 1:
                if self.accept_inverted_pairs:
                    # Technically falling to rising pairs should not be acceptable
                    time_diff = second_edge.time - first_edge.time
                    time_diffs.append(time_diff)
                    self.echo_stack.remove(first_edge)
                    self.echo_stack.remove(second_edge)
                else:
                    self.echo_stack.remove(first_edge)
            elif first_edge.mode == 1 and second_edge.mode == 1:
                # Two rising edges in a row, we should remove the first
                self.echo_stack.remove(first_edge)
            else:
                # Two falling edges in a row, remove both.
                self.echo_stack.remove(first_edge)
                self.echo_stack.remove(second_edge)

        # Clear last element if falling edge
        if len(self.echo_stack) == 1 and not self.accept_inverted_pairs:
            if self.echo_stack[0].mode == 0:
                self.echo_stack.remove(self.echo_stack[0])

        return np.array(time_diffs)


def main():
    """ Simple main loop for testing """
    args = parse_args()

    GPIO.setmode(GPIO.BCM)
    sensor = HY_SRF05(
        args.trigger_pin, args.echo_pin,
        trigger_sleep=args.trigger_sleep,
        start_trigger_thread=args.trigger_thread,
        accept_inverted_pairs=args.accept_inverted_pairs)

    if args.trigger_thread:
        while True:
            time.sleep(args.trigger_sleep * 3)
            distance = sensor.get_distance(send_trigger=False)
            if distance:
                print(f"Distance: {distance:1.4f} m")
    else:
        while True:
            distance = sensor.get_distance(send_trigger=True)
            if distance:
                print(f"Distance: {distance:1.4f} m")


def parse_args():
    parser = argparse.ArgumentParser(description="Test program for HY-SRF05 sensor.")
    parser.add_argument(
        '-t', '--trigger-pin', type=int, default=3,
        help='GPIO pin connected to the HY-SRF05 trigger pin.')
    parser.add_argument(
        '-e', '--echo-pin', type=int, default=2,
        help='GPIO pin connected to the HY-SRF05 echo pin.')
    parser.add_argument(
        '-s', '--trigger-sleep', type=float, default=0.05,
        help='Minimum allowed time between sensor triggers. Minimum allowed is 0.03 s.')
    parser.add_argument(
        '-i', '--accept-inverted-pairs', action='store_true',
        help='Choose to accept falling to rising echo pairs as distance measurements.')
    parser.add_argument(
        '-d', '--trigger-thread', action='store_true',
        help='Start a thread to trigger sensor automatically.')
    return parser.parse_args()


if __name__ == '__main__':
    main()
