"""Config."""

from pathlib import Path
_CURRENT_DIR = Path(__file__).resolve().parent


def _get_task_config():
    config = dict(
        constructor=dict(
            module='tasks.cursor_control',
            method='CursorControl',
        ),
        kwargs=dict(
            sampler_start_train_kwargs=dict(
                area_keep=((-1, -1), (0, 0)),
            ),
            sampler_goal_train_kwargs=dict(
                area_keep=((0, 0), (1, 1)),
            ),
            sampler_start_test_kwargs=dict(
                area_keep=((0, 0), (1, 1)),
            ),
            sampler_goal_test_kwargs=dict(
                area_keep=((-1, -1), (0, 0)),
            ),
            max_action_magnitude=0.1,
        ),
    )
    return config


def _get_model_config():
    config = dict(
        constructor=dict(
            module='models.monolithic',
            method='Student',
        ),
        kwargs=dict(
            teacher_log_dir=str((_CURRENT_DIR / "../training/logs/cursor_control_teacher_v0").resolve()),
            reg_coeff=1.,
            task=_get_task_config(),
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
            training_steps=1000,
            batch_size=128,
        ),
        'random_seed': 0,
    }
    return config
