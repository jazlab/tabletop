"""Config."""

import numpy as np


def _get_task_config():
    config = dict(
        constructor=dict(
            module='tasks.arm_2dof',
            method='Arm2DOF',
        ),
        kwargs=dict(
            segment_lengths=(1.5, 1),
            goal_theta=(0.5 * np.pi, 2 * np.pi),
            goal_theta_test=(0, 0.5 * np.pi),
            max_action_magnitude=0.1,
        ),
    )
    return config


def _get_model_config():
    config = dict(
        constructor=dict(
            module='models.teacher',
            method='Teacher',
        ),
        kwargs=dict(
            task=_get_task_config(),
            hidden_features=[1024, 1024],
            action_scale=0.2,
        ),
    )
    return config


def get_config():
    config = {
        'constructor': dict(
            module='trainer',
            method='Trainer',
        ),
        'kwargs': dict(
            model=_get_model_config(),
            training_steps=5000,
            batch_size=512,
        ),
        'random_seed': 0,
    }
    return config
