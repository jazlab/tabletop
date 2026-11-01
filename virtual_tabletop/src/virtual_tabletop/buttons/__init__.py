"""Response buttons on the task objects (RL model).

Each distinct object geometry (``small_object``, ``big_object``) carries a
button; pressing it is the response in the button RL task.

- ``config/buttons.yaml``: where each button goes (object frame), and the
  hollow collision shape for the cup;
- :mod:`.config`: loads it;
- :mod:`.model`: adds the buttons (and hollow collision) to the RL model's
  stimulus bodies.

Choosing placements: ``vtt-object-sheets`` and ``vtt-button-reach``
(:mod:`virtual_tabletop.design`).
"""
