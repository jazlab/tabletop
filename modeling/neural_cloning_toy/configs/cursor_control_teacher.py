"""Config."""


def _get_task_config():
    config = dict(
        constructor=dict(
            module='tasks.cursor_control',
            method='CursorControl',
        ),
        kwargs=dict(
            sampler_start_train_kwargs=dict(
                area_keep=((-1, -1), (1, 1)),
            ),
            sampler_goal_train_kwargs=dict(
                area_keep=((-1, -1), (1, 1)),
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
            method='Teacher',
        ),
        kwargs=dict(
            task=_get_task_config(),
            hidden_features=[512],
            action_scale=0.1,
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
