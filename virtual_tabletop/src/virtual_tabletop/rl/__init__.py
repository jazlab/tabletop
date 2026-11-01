"""The RL tasks: a Gymnasium env, its scene and trials, scripted policies.

- :mod:`.env`: ``PresentationEnv``, the three tasks (none / touch / button);
- :mod:`.scene`: the RL model's presented object, buttons and contacts;
- :mod:`.trials`: trial sequence and phase timing;
- :mod:`.policies`: scripted baselines (reach, button press);
- :mod:`.press_paths`: wall-free arm paths onto the buttons;
- :mod:`.demo`: ``vtt-rl-demo`` (viewer or video).

Tools for *deciding* the RL setup live in :mod:`virtual_tabletop.design`.
"""
