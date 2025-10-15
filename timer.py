# mytimer.py
from datetime import datetime

from globals import *

class MyTimer:
    """
    timer.start() - should start the timer
    timer.pause() - should pause the timer
    timer.resume() - should resume the timer
    timer.get() - should return the current time
    timer.is_done() - return True if the timer is done
    """

    def __init__(self, ceil):
        self._time_started = None
        self._time_paused = None
        self._paused = False
        self._ceil = ceil

    def start(self):
        """ Starts an internal timer by recording the current time """
        self._time_started = datetime.now()

    def pause(self):
        """ Pauses the timer """
        if self._time_started is None:
            raise ValueError("Timer not started")
        if self._paused:
            raise ValueError("Timer is already paused")
        self._time_paused = datetime.now()
        self._paused = True

    def resume(self):
        """ Resumes the timer by adding the pause time to the start time """
        if self._time_started is None:
            raise ValueError("Timer not started")
        if not self._paused:
            raise ValueError("Timer is not paused")
        pause_time = datetime.now() - self._time_paused
        self._time_started += pause_time
        self._paused = False

    def get(self):
        """ Returns a timedelta object showing the amount of time
            elapsed since the start time, less any pauses """
        if self._time_started is None:
            raise ValueError("Timer not started")
        if self._paused:
            return self._time_paused - self._time_started
        else:
            return datetime.now() - self._time_started

    def is_done(self):
        return self._get().total_seconds() > self._ceil