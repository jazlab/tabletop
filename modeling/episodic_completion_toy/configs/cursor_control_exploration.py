"""Config."""


def _get_agent_config():
    config = dict(
        constructor=dict(
            module='tasks.cursor_control',
            method='ExplorationAgent',
        ),
        kwargs=dict(
            environment=dict(
                constructor=dict(
                    module='tasks.cursor_control',
                    method='CursorControlEnvironment',
                ),
                kwargs=dict(
                    batch_size=512,
                    feedback_window=0.4,
                    bounds=(-1, 1),
                ),
            ),
            scale=1,
            smooth_tau=0.9,
            seq_length=1,
            burn_in_steps=0,
        ),
    )
    return config


def _get_dataset_config():
    config = dict(
        constructor=dict(
            module='dataset',
            method='Dataset',
        ),
        kwargs=dict(
            agent=_get_agent_config(),
            snr_logit_range=(-2000, 2000),
            # snr_logit_range=(5, 10),
        ),
    )
    return config


def _get_model_config():
    config = dict(
        constructor=dict(
            module='models.monolithic_exploration',
            method='MonolithicExploration',
        ),
        kwargs=dict(
            dataset=_get_dataset_config(),
            hidden_features=[512],
            n_rnn_steps=5,
            tau=0.5,
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
            training_steps=2000,
        ),
        'random_seed': 0,
    }
    return config
