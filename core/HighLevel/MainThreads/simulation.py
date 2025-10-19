# core/HighLevel/MainThreads/simulation.py
import threading
import numpy as np
from PySide6.QtCore import Signal, QObject

from ...HighLevel.Orchestration.orchestrators import Follow

from ...LowLevel.Timing.timer import MyTimer
from ...LowLevel.Timing.fps import BasicFPS

from ...LowLevel.Utilities.helpers import print_for_gui
from ...LowLevel.Utilities.globals import CONFIG, LOGGER, ENVIRONMENT
from ...LowLevel.Utilities.decorators import safe_call

from ...LowLevel.RuntimeOutputs.plots import plot



class BasicSimulationRunner:
    def __init__(self, orchestrator):
        LOGGER.info("\tSimulation: Initiating")
        self._orchestrator = orchestrator()
        self._running_event = threading.Event()
        self._running_event.set()
        self._running_event.clear()
        self._terminated = False
        self._fps = BasicFPS()
        self._timer = MyTimer(CONFIG["simulation"]["timer_ceil"])

    @property
    def orchestrator(self):
        return self._orchestrator

    def is_running(self):
        return self._running_event.is_set()

    def is_terminated(self):
        return self._terminated == True

    def pause(self):
        self._timer.pause()
        self._running_event.clear()
        LOGGER.debug("Simulation: Pause")

    def resume(self):
        self._timer.resume()
        self._running_event.set()
        LOGGER.debug("Simulation: Resume")

    def terminate(self):
        if not self._timer.paused:
            self._timer.pause()
        self._terminated = True
        LOGGER.debug("Simulation: Terminate")

    def status(self):
        raise NotImplementedError("Subclasses should implement this method")

    def run(self):
        raise NotImplementedError("Subclasses should implement this method")


class SimulationRunner(QObject, BasicSimulationRunner):
    status_ready = Signal(str)

    def __init__(self, orchestrator= Follow):
        super().__init__(orchestrator=orchestrator)
        LOGGER.info(f"\tSimulation: Initiated {self.__class__.__name__}")

    # Camera Streamer helpers
    def continue_streaming(self):
        return not self.is_terminated() and self._running_event.wait()

    def stream(self, frame):
        self._orchestrator.stream(frame)

    # GUI helpers
    def set_pad_vel(self, vel):
        self._orchestrator.set_pad_vel(vel)

    def get_pad_vel(self):
        return self._orchestrator.get_pad_vel()

    def status(self):
        status = f"{self.__class__.__name__} status:"
        status += str(self._fps)
        status += str(self._timer)
        status += self._orchestrator.status()
        if self._orchestrator.scene_ended():
            status += "\nScene ended:"
            logs_dict = self._orchestrator.get_logs()
            drone_log, pad_log = logs_dict["Quadrotor"], logs_dict["Pad"]

            delta_time = drone_log["Time (sec)"][-1] - drone_log["Time (sec)"][0]
            status += f"\ndelta time (s): {print_for_gui(delta_time)}"

            drone_end_pos = np.array([drone_log["x_true"][-1], drone_log["y_true"][-1]])
            pad_end_pos = np.array([pad_log["x_true"][-1], pad_log["y_true"][-1]])
            delta_pos = drone_end_pos - pad_end_pos
            status += f"\ndelta pos: {print_for_gui(delta_pos)}"

        return status

    # app.py
    def plot_logs(self):
        self._orchestrator.plot_logs()

    @safe_call()
    def run(self):
        LOGGER.debug("Simulation: Running")
        self._timer.start()
        self._timer.pause()
        while not self.is_terminated() and self._running_event.wait() and not self._timer.is_done():
            self._orchestrator.step_scene()  # advance scene
            self.status_ready.emit(self.status())  # emit status
            ENVIRONMENT.step()  # advance physics
            self._fps.maintain()
        LOGGER.info("Simulation: Terminated")
