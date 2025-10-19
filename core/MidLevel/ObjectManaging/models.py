# core/MidLevel/ObjectManaging/models.py
from ...MidLevel.ObjectManaging.sensors import GPS, IMU, Rangefinder, Touch

from ...LowLevel.RuntimeOutputs.logs import Log
from ...LowLevel.Utilities.helpers import print_for_gui
from ...LowLevel.Utilities.globals import CONFIG, LOGGER, ENVIRONMENT


class BasicModel:
    def __init__(self, child_class_name, xml_name):
        self._xml_name = xml_name
        self._body_id = ENVIRONMENT.body_id(xml_name)
        self._sensors = dict()
        self._sensors["gps"] = GPS(
            pos_sensor_name=CONFIG[child_class_name]["sensors"]["pos"],
            vel_sensor_name=CONFIG[child_class_name]["sensors"]["vel"],
        )
        self._log = Log(name=f"Log_{child_class_name}")
        ENVIRONMENT.set_body_cda(
            body=self._xml_name, cda=CONFIG[child_class_name]["cda"]
        )

    @property
    def body_id(self):
        return self._body_id

    @property
    def sensors(self):
        return self._sensors

    @property
    def log(self):
        return self._log

    def plot_log(self):
        self._log.plot()

    def get_pos(self):
        return self.sensors["gps"].get_pos()

    def get_vel(self):
        return self.sensors["gps"].get_vel()

    def get_true_pos(self):
        return ENVIRONMENT.world_pos_of_body(self._body_id)

    def get_true_vel(self):
        return ENVIRONMENT.world_linvel_of_body(self._body_id)

    def update_log(self):
        t = ENVIRONMENT.get_time()
        x_true, y_true, z_true = self.get_true_pos()
        vx_true, vy_true, vz_true = self.get_true_vel()
        x, y, z = self.get_pos()
        vx, vy, vz = self.get_vel()

        # Append values to the log
        self._log.append_time(t)
        self._log.append_channels(
            x_true=x_true, y_true=y_true, z_true=z_true,
            vx_true=vx_true, vy_true=vy_true, vz_true=vz_true,
            x=x, y=y, z=z,
            vx=vx, vy=vy, vz=vz,
        )

    def status(self):
        raise NotImplementedError("Subclasses should implement this method")


class Quadrotor(BasicModel):
    def __init__(self):
        super().__init__(
            child_class_name=self.__class__.__name__,
            xml_name=CONFIG["Quadrotor"]["xml_body_name"],
        )
        self._sensors["rangefinder"] = Rangefinder(
            range_sensor_name=CONFIG["Quadrotor"]["sensors"]["rangefinder"]
        )
        self._sensors["imu"] = IMU(
            framequat_sensor_name=CONFIG["Quadrotor"]["sensors"]["framequat"],
            gyro_sensor_name=CONFIG["Quadrotor"]["sensors"]["gyro"],
            accelerometer_sensor_name=CONFIG["Quadrotor"]["sensors"]["accelerometer"],
        )
        self._actuator_ids, self._actuator_names = ENVIRONMENT.actuators_for_body(
            self._body_id
        )
        LOGGER.info(f"\t\t\tModel: Initiated {self.__class__.__name__}")

    @property
    def actuator_ids(self):
        return self._actuator_ids

    @property
    def actuator_names(self):
        return self._actuator_names

    def get_orientation(self):
        return self._sensors["imu"].get_orientation()

    def get_gyro(self):
        return self._sensors["imu"].get_gyro()

    def get_accelerometer(self):
        return self._sensors["imu"].get_accelerometer()

    def get_height(self):
        return self._sensors["rangefinder"].get()

    def update_log(self):
        super().update_log()
        h_rf = self.get_height()
        gx, gy, gz = self.get_gyro()
        r, p, y = self.get_orientation()
        accx, accy, accz = self.get_accelerometer()

        # Append values to the log
        self._log.append_channels(
            rangefinder=h_rf,
            gyro_x=gx, gyro_y=gy, gyro_z=gz,
            orientation_x_roll=r, orientation_y_pitch=p, orientation_z_yaw=y,
            acceleration_x=accx, acceleration_y=accy, acceleration_z=accz,
        )

    def status(self):
        status = f"{self.__class__.__name__} \ttrue status:"
        status += f"\tpos: {print_for_gui(self.get_true_pos())}"
        status += f"\tvel: {print_for_gui(self.get_true_vel())}\n"
        for sensor_obj in self._sensors.values():
            status += f"\t\t{sensor_obj.status()}\n"
        return status


class Pad(BasicModel):
    def __init__(self):
        super().__init__(
            child_class_name=self.__class__.__name__,
            xml_name=CONFIG["Pad"]["xml_body_name"],
        )
        self._sensors["touch"] = Touch(
            touch_sensor_name=CONFIG["Pad"]["sensors"]["touch"]
        )

        self._radius = CONFIG["Pad"]["radius"]
        self._joint_x_name = "Pad_joint_x"
        self._joint_y_name = "Pad_joint_y"
        LOGGER.info(f"\t\t\tModel: Initiated {self.__class__.__name__}")

    @property
    def radius(self):
        return self._radius

    @property
    def joint_x_name(self):
        return self._joint_x_name

    @property
    def joint_y_name(self):
        return self._joint_y_name

    def get_touch_force(self):
        return self._sensors["touch"].get()

    def update_log(self):
        super().update_log()
        tf = self.get_touch_force()

        # Append values to the log
        self._log.append_channels(touch=tf)

    def status(self):
        status = f"{self.__class__.__name__}\t\ttrue status:"
        status += f"\tpos: {print_for_gui(self.get_true_pos())}"
        status += f"\tvel: {print_for_gui(self.get_true_vel())}\n"
        for sensor_obj in self._sensors.values():
            status += f"\t\t{sensor_obj.status()}\n"
        return status
