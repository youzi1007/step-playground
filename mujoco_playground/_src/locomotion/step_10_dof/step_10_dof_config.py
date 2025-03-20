class Step10DofConfig:
    def __init__(self):
        self.urdf_path = "mujoco_playground/resources/robots/STEP_10_DOF/urdf/STEP-10-DOF.urdf"
        self.reward_config = {
            "tracking_lin_vel": 1.0,
            "torques": -0.01
        }
        self.action_repeat = 1  # Default
