import cv2
import numpy as np
from PySide6.QtCore import Signal, QObject

from ...LowLevel.Timing.fps import BasicFPS
from ...LowLevel.Utilities.helpers import print_for_gui
from ...LowLevel.Utilities.globals import CONFIG, LOGGER, ENVIRONMENT


class CameraStreamer(QObject):
    frame_ready = Signal(np.ndarray)
    status_ready = Signal(str)

    def __init__(self, simulation):
        super().__init__()
        self._simulation = simulation
        self._fps = BasicFPS(frequency= CONFIG["camera_streamer"]["frequency"])
        self._resolutions = CONFIG["camera_streamer"]["resolutions"]
        self._camera_name = CONFIG["camera_streamer"]["name"]

        LOGGER.info(f"\tCameraStreamer: Initiated {self.__class__.__name__}")

    def status(self):
        status = f"{self.__class__.__name__} status:"
        status += f"\tname: {self._camera_name} status:"
        status += f"\t\tcurrent fps: {print_for_gui(self._fps.curr_fps)}"
        status += f"\ttarget fps: {print_for_gui(self._fps.target_fps)}"
        return status

    def run(self):
        ENVIRONMENT.init_camera(self._camera_name, tuple(self._resolutions["high"]))

        while self._simulation.continue_streaming():

            rgb_image = ENVIRONMENT.render_camera(self._camera_name)

            # stream to simulation in high resolution
            self._simulation.stream(frame=rgb_image)  # stream frame to orchestrator

            # emit to gui in low resolution
            gui_frame = cv2.resize(rgb_image, tuple(self._resolutions["low"]), interpolation=cv2.INTER_AREA)
            self.frame_ready.emit(gui_frame)  # emit frame to gui
            self.status_ready.emit(self.status())  # emit status to gui

            self._fps.maintain()

        ENVIRONMENT.close_camera(self._camera_name)
