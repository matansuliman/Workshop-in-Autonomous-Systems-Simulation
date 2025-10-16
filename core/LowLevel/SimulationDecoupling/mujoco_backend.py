# core/LowLevel/SimulationDecoupling/mujoco_backend.py
from __future__ import annotations
from typing import Iterable, Optional
import numpy as np
import mujoco

from ...LowLevel.SimulationDecoupling.backend import PhysicsBackend


class MujocoBackend(PhysicsBackend):
    """Concrete implementation of PhysicsBackend using MuJoCo."""

    def __init__(self, xml_path: str):
        self._model = mujoco.MjModel.from_xml_path(xml_path)
        self._data = mujoco.MjData(self._model)

    # --- core properties ----
    @property
    def model(self) -> mujoco.MjModel:
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        return self._data

    # ---- core timing ----
    @property
    def dt(self) -> float:
        return self._model.opt.timestep

    def get_time(self) -> float:
        return self._data.time

    # ---- stepping ----
    def step_once(self) -> None:
        mujoco.mj_step(self._model, self._data)

    # ---- external forces ----
    def apply_force(self, body: str | int, force_world: Iterable[float], torque_world: Optional[Iterable[float]] = None,
                    *, mode: str = "add", ) -> None:
        """
        Apply an external spatial force (world frame) on a body COM for the current step.
        If mode == 'set' it overwrites; if 'add' it accumulates.
        """
        f = np.asarray(force_world, dtype=float).reshape(3)
        t = np.asarray(torque_world or (0.0, 0.0, 0.0), dtype=float).reshape(3)
        bid = self._model.body(body).id if isinstance(body, str) else int(body)

        if mode == "set":
            self._data.xfrc_applied[bid, :3] = f
            self._data.xfrc_applied[bid, 3:] = t
        elif mode == "add":
            self._data.xfrc_applied[bid, :3] += f
            self._data.xfrc_applied[bid, 3:] += t
        else:
            raise ValueError("mode must be 'set' or 'add'")

    def clear_external_forces(self) -> None:
        """Zero out all xfrc_applied for this step (safe to call every step)."""
        self._data.xfrc_applied[:, :] = 0.0

    # ---- world parameters ----
    def set_gravity(self, g_world: Iterable[float]) -> None:
        self._model.opt.gravity[:] = np.asarray(g_world, dtype=float).reshape(3)

    def set_air_density(self, rho: float) -> None:
        self._model.opt.density = float(rho)

    def set_viscosity(self, mu: float) -> None:
        self._model.opt.viscosity = float(mu)

    def set_timestep(self, dt: float) -> None:
        self._model.opt.timestep = float(dt)

    # ---- Reset ----
    def reset(self, qpos: Optional[Iterable[float]] = None, qvel: Optional[Iterable[float]] = None) -> None:
        """Reset simulation and optionally override qpos/qvel."""
        mujoco.mj_resetData(self._model, self._data)
        if qpos is not None:
            qpos = np.asarray(qpos, dtype=float)
            if qpos.size != self._model.nq:
                raise ValueError(f"qpos size {qpos.size} != {self._model.nq}")
            self._data.qpos[:] = qpos
        if qvel is not None:
            qvel = np.asarray(qvel, dtype=float)
            if qvel.size != self._model.nv:
                raise ValueError(f"qvel size {qvel.size} != {self._model.nv}")
            self._data.qvel[:] = qvel
        mujoco.mj_forward(self._model, self._data)

    # ---- Body / Joint identifiers ----
    def body_id(self, body: str | int) -> int:
        if isinstance(body, int):
            return int(body)
        return self._model.body(body).id

    def body_name(self, body: str | int) -> str:
        if isinstance(body, str):
            return body
        return self._model.body(body).name

    def joint_id(self, joint: str | int) -> int:
        if isinstance(joint, int):
            return int(joint)
        return self._model.joint(joint).id

    def actuator_id(self, actuator: str | int) -> int:
        if isinstance(actuator, int):
            return int(actuator)
        return self._model.actuator(actuator).id

    def sensor_id(self, sensor: str | int) -> int:
        if isinstance(sensor, int):
            return int(sensor)
        return self._model.sensor(sensor).id

    def sensor_name(self, sensor: str | int) -> str:
        if isinstance(sensor, int):
            return str(sensor)
        return self._model.sensor(sensor).name

    # ---- Body state access ----
    def get_body_pos(self, body: str | int) -> np.ndarray:
        """World-frame COM position."""
        return self._data.xpos[self.body_id(body)].copy()

    def get_body_linvel(self, body: str | int) -> np.ndarray:
        """World-frame COM linear velocity."""
        bid = self.body_id(body)
        r_bw = self._data.xmat[bid].reshape(3, 3)
        v_lin_body = self._data.cvel[bid, 3:]
        return r_bw @ v_lin_body

    # ---- Joint and actuator controls ----
    def set_joint_qpos(self, joint: str | int, value: float):
        jid = self.joint_id(joint)
        adr = self._model.jnt_qposadr[jid]
        self._data.qpos[adr] = float(value)

    def set_joint_qvel(self, joint: str | int, value: float):
        jid = self.joint_id(joint)
        adr = self._model.jnt_dofadr[jid]
        self._data.qvel[adr] = float(value)

    def set_ctrl(self, actuator: str | int, value: float):
        aid = self.actuator_id(actuator)
        self._data.ctrl[aid] = float(value)

    def set_ctrls(self, mapping: dict[str, float]):
        for name, val in mapping.items():
            self.set_ctrl(name, val)

    # ---- Viewer ----
    def launch_viewer(self):
        import glfw, mujoco.viewer
        glfw.init()
        return mujoco.viewer.launch_passive(self._model, self._data)


    # ---- Setters ----
    def set_free_body_pose(self, body: str | int,
                           pos_world: Optional[Iterable[float]] = None, quat_wxyz: Optional[Iterable[float]] = None):
        bid = self.body_id(body)
        jid = self._model.body_jntnum[bid]
        if jid <= 0:
            raise ValueError(f"Body '{self.body_name(bid)}' has no joint or not freejoint.")
        jadr = self._model.jnt_qposadr[self._model.body_jntadr[bid]]
        jtype = self._model.jnt_type[self._model.body_jntadr[bid]]
        if jtype != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError(f"Body '{self.body_name(bid)}' primary joint is not free.")

        if pos_world is not None:
            self._data.qpos[jadr:jadr + 3] = np.asarray(pos_world, dtype=float).reshape(3)
        if quat_wxyz is not None:
            self._data.qpos[jadr + 3:jadr + 7] = np.asarray(quat_wxyz, dtype=float).reshape(4)

    def set_free_body_velocity(self, body: str | int,
                               linvel_world: Optional[Iterable[float]] = None, angvel_world: Optional[Iterable[float]] = None):
        bid = self.body_id(body)
        jid = self._model.body_jntnum[bid]
        if jid <= 0:
            raise ValueError(f"Body '{self.body_name(bid)}' has no joint or not freejoint.")
        vadr = self._model.jnt_dofadr[self._model.body_jntadr[bid]]
        jtype = self._model.jnt_type[self._model.body_jntadr[bid]]
        if jtype != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError(f"Body '{self.body_name(bid)}' primary joint is not free.")

        if angvel_world is not None:
            self._data.qvel[vadr:vadr + 3] = np.asarray(angvel_world, dtype=float).reshape(3)
        if linvel_world is not None:
            self._data.qvel[vadr + 3:vadr + 6] = np.asarray(linvel_world, dtype=float).reshape(3)


    # ---- Getters ----
    def actuators_for_body(self, body: int | str):
        """
        Return actuator ids and names that act on the given body.
        Works for JOINT-driven and SITE-driven actuators.
        """
        if isinstance(body, str):
            body_id = self.body_id(body)
        else:
            body_id = int(body)

        mdl = self._model
        ids, names = [], []

        for aid in range(mdl.nu):
            trntype = int(mdl.actuator_trntype[aid])
            idx0 = int(mdl.actuator_trnid[aid, 0])

            # JOINT-driven
            if trntype == mujoco.mjtTrn.mjTRN_JOINT:
                if 0 <= idx0 < mdl.njnt and mdl.jnt_bodyid[idx0] == body_id:
                    ids.append(aid)
                    names.append(mdl.actuator(aid).name)
                continue

            # SITE-driven (e.g. quadrotor motors)
            if trntype == mujoco.mjtTrn.mjTRN_SITE:
                if 0 <= idx0 < mdl.nsite and mdl.site_bodyid[idx0] == body_id:
                    ids.append(aid)
                    names.append(mdl.actuator(aid).name)
                continue

        return ids, names

    def get_gravity(self):
        return self._model.opt.gravity.copy()


    # ---- Sensors ----
    def get_sensor_metadata(self, sensor_name: str) -> tuple[int, int]:
        sid = self.sensor_id(sensor_name)
        adr = self._model.sensor_adr[sid]
        dim = self._model.sensor_dim[sid]
        return adr, dim

    def get_sensor_data(self, sensor_name: str) -> np.ndarray:
        adr, dim = self.get_sensor_metadata(sensor_name)
        return self._data.sensordata[adr: adr + dim].copy()

    # ---- Camera ----
    def render_camera(self, camera_name: str, resolution: tuple[int, int]) -> np.ndarray:
        import glfw

        # Initialize offscreen context once per backend instance
        if not glfw.init():
            raise RuntimeError("GLFW initialization failed")

        width, height = resolution
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        window = glfw.create_window(width, height, "", None, None)
        glfw.make_context_current(window)

        # Create camera and scene
        camera = mujoco.MjvCamera()
        option = mujoco.MjvOption()
        scene = mujoco.MjvScene(self._model, maxgeom=1000)
        context = mujoco.MjrContext(self._model, mujoco.mjtFontScale.mjFONTSCALE_150)

        cam_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
        camera.fixedcamid = cam_id

        # Update and render
        mujoco.mjv_updateScene(self._model, self._data, option, None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene)
        mujoco.mjr_render(mujoco.MjrRect(0, 0, width, height), scene, context)

        rgb = np.zeros((height, width, 3), dtype=np.uint8)
        mujoco.mjr_readPixels(rgb, None, mujoco.MjrRect(0, 0, width, height), context)
        rgb = np.flip(rgb, axis=0)

        glfw.destroy_window(window)
        return rgb







