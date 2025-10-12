# mytimer.py
from datetime import datetime
import time

class MyTimer():
    """
    timer.start() - should start the timer
    timer.pause() - should pause the timer
    timer.resume() - should resume the timer
    timer.get() - should return the current time
    """

    def __init__(self):
        print('Initializing timer')
        self.timestarted = None
        self.timepaused = None
        self.paused = False

    def start(self):
        """ Starts an internal timer by recording the current time """
        print("Starting timer")
        self.timestarted = datetime.now()

    def pause(self):
        """ Pauses the timer """
        if self.timestarted is None:
            raise ValueError("Timer not started")
        if self.paused:
            raise ValueError("Timer is already paused")
        print('Pausing timer')
        self.timepaused = datetime.now()
        self.paused = True

    def resume(self):
        """ Resumes the timer by adding the pause time to the start time """
        if self.timestarted is None:
            raise ValueError("Timer not started")
        if not self.paused:
            raise ValueError("Timer is not paused")
        print('Resuming timer')
        pausetime = datetime.now() - self.timepaused
        self.timestarted = self.timestarted + pausetime
        self.paused = False

    def get(self):
        """ Returns a timedelta object showing the amount of time
            elapsed since the start time, less any pauses """
        print('Get timer value')
        if self.timestarted is None:
            raise ValueError("Timer not started")
        if self.paused:
            return self.timepaused - self.timestarted
        else:
            return datetime.now() - self.timestarted

if __name__ == "__main__":
    t = MyTimer()
    t.start()
    print('Waiting 2 seconds'); time.sleep(2)
    print(t.get())
    print('Waiting 1 second'); time.sleep(1)
    t.pause()
    print('Waiting 2 seconds [paused]'); time.sleep(2)
    print(t.get())
    print('Waiting 1 second [paused]'); time.sleep(1)
    print(t.get())
    print('Waiting 1 second [paused]'); time.sleep(1)
    t.resume()
    print('Waiting 1 second'); time.sleep(1)
    print(t.get())