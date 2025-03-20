# Copyright 2025 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0
"""STEP 10-DOF Locomotion Environment for Mujoco Playground."""

import numpy as np
import jax
import jax.numpy as jnp
from mujoco import mjx
from mujoco_playground._src import mjx_env
from mujoco_playground._src.locomotion.step_10_dof_config import Step10DofConfig

class Step10DofEnv(mjx_env.MjxEnv):
    """STEP 10-DOF Locomotion Environment using Mujoco Playground."""

    def __init__(self, config: Step10DofConfig, config_overrides=None):
        super().__init__(config=config, config_overrides=config_overrides)

        # Load Mujoco model from URDF
        self.model = mjx.load_model_from_path(config.urdf_path)
        self.state = mjx.get_initial_state(self.model)

    def reset(self, rng: jax.Array) -> mjx_env.State:
        """Reset the environment to initial state."""
        rng, sub_rng = jax.random.split(rng)
        self.state = mjx.get_initial_state(self.model)
        self.state.qpos = self.config.init_qpos
        self.state.qvel = jnp.zeros(self.model.nv)
        return self.state

    def step(self, state: mjx_env.State, action: jax.Array) -> mjx_env.StepResult:
        """Perform a simulation step."""
        # Apply control inputs
        state.ctrl = jnp.clip(action, -1.0, 1.0) * self.config.action_scale
        next_state = mjx.step(self.model, state)

        # Compute reward
        reward = self._compute_reward(next_state)

        # Check termination condition
        done = self._check_done(next_state)

        return mjx_env.StepResult(next_state, reward, done, {})

    def _compute_reward(self, state: mjx_env.State) -> float:
        """Compute reward based on velocity tracking and energy consumption."""
        forward_velocity = state.qvel[0]
        torque_penalty = jnp.sum(jnp.abs(state.ctrl)) * self.config.torque_penalty
        return forward_velocity - torque_penalty

    def _check_done(self, state: mjx_env.State) -> bool:
        """Determine if the episode should end."""
        return state.qpos[2] < self.config.termination_height

    def observation(self, state: mjx_env.State) -> jax.Array:
        """Return the observation as a concatenation of qpos and qvel."""
        return jnp.concatenate([state.qpos, state.qvel])
