"""Tools for deciding the RL setup: each informs one design decision.

Nothing in the runtime packages imports these.

| Tool (CLI) | Module | Decision it informs |
|---|---|---|
| ``vtt-arm-explorer`` | :mod:`.arm_explorer` | joints and ranges: pose the arms with degree sliders, read angles and object gaps |
| ``vtt-arm-report`` | :mod:`.arm_report` | joints and ranges: sweep videos, fingertip workspace, joint ablation (which joints can be frozen) |
| ``vtt-object-sheets`` | :mod:`.object_sheets` | button placement: each object geometry drawn in its own frame (cm grid) |
| ``vtt-button-reach`` | :mod:`.button_reach` | shoulder placement: can each button be pressed without going through its object; scan shoulder offsets |
"""
