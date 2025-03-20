import mujoco
import numpy as np

class Step10DofEnv:
    def __init__(self, urdf_path="mujoco_playground/_src/locomotion/STEP_10_DOF/urdf/STEP-10-DOF.urdf"):
        self.model = mujoco.MjModel.from_xml_path(urdf_path)
        self.data = mujoco.MjData(self.model)

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        return self._get_obs()

    def step(self, action):
        self.data.ctrl[:] = action
        mujoco.mj_step(self.model, self.data)
        obs = self._get_obs()
        reward = self._compute_reward()
        done = self._check_done()
        return obs, reward, done, {}

    def _get_obs(self):
        # Combine positions and velocities into one observation array
        return np.concatenate([self.data.qpos, self.data.qvel])

    def _compute_reward(self):
        # Forward velocity as positive reward, control energy as penalty
        forward_velocity = self.data.qvel[0]
        energy_penalty = np.sum(np.abs(self.data.actuator_force))
        return forward_velocity - 0.01 * energy_penalty

    def _check_done(self):
        # End the episode if the robot falls (z-height too low)
        return self.data.qpos[2] < 0.2
