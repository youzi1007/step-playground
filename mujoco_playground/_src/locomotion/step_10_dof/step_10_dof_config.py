# Copyright 2025 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0
"""Configuration for STEP 10-DOF Locomotion Environment."""

from ml_collections import config_dict

def Step10DofConfig():
    """Configuration dictionary for the STEP 10-DOF locomotion environment."""
    config = config_dict.ConfigDict()

    # URDF Model Path
    config.urdf_path = "mujoco_playground/resources/robots/STEP_10_DOF/urdf/STEP-10-DOF.urdf"

    # Initial Joint Positions
    config.init_qpos = [0.0] * 10  # 10-DOF initial state

    # Action Scaling
    config.action_scale = 1.0

    # Reward Weights
    config.torque_penalty = 0.01

    # Termination Condition
    config.termination_height = 0.2

    return config
