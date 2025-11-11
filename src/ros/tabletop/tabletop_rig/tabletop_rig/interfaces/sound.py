import asyncio
import os
from collections.abc import (
    Mapping,
)
from typing import Any, Optional

from mingus.containers import Note
from mingus.midi import fluidsynth

from tabletop_rig.interfaces.base import BaseInterface
from tabletop_rig.nodes.base import BaseNode


class SoundInterface(BaseInterface):
    ###########################################################################
    ########## Initialization #################################################
    ###########################################################################

    def __init__(self, node: BaseNode):
        """Initializes the DashboardInterface"""
        super().__init__(node, "sound_interface")

        config: dict[str, Any] = self.node.get_parameter_wrapper("sound")

        self.enabled = config["enable"]
        if not self.enabled:
            return

        soundfont_path = os.path.expandvars(config["soundfont_path"])
        if not os.path.exists(soundfont_path):
            raise FileNotFoundError(f"Soundfont {soundfont_path} not found")

        if not fluidsynth.init(soundfont_path, driver="pulseaudio"):
            raise RuntimeError("Failed to initialize fluidsynth")

        self._default_note = Note(**config["default_note"])
        if (
            not isinstance(config["default_duration"], (int, float))
            or config["default_duration"] <= 0
        ):
            raise ValueError(
                f"Default duration must be a positive number, got {config['default_duration']}"
            )
        self._default_duration = float(config["default_duration"])

        fluidsynth.set_instrument(
            channel=self._default_note.channel, midi_instr=config["instrument"]
        )

        self.log("Sound interface initialized")

    def init_sound(self):
        """Initialize sound."""

    def start_note(self, note: Note):
        """Start a note."""
        if self.enabled:
            fluidsynth.play_Note(note)

    def stop_note(self, note: Note):
        """Stop a note."""
        if self.enabled:
            fluidsynth.stop_Note(note)

    async def play_sound(
        self,
        note: Optional[Note | Mapping[str, Any]] = None,
        duration: Optional[float] = None,
    ):
        """Play a sound for a given duration.

        Args:
            note: Note to play. If None, the default note is used.
            instrument: Midi instrument to play, e.g. 62. If None, the default instrument is used.
            duration: Duration of the sound in seconds. If None, the default duration is used.
        """
        if self.enabled:
            if note is None:
                note = self._default_note
            elif not isinstance(note, Note):
                note = Note(**note)
            note.channel = self._default_note.channel

            if duration is None:
                duration = self._default_duration

            fluidsynth.play_Note(note)
            await asyncio.sleep(duration)
            fluidsynth.stop_Note(note)
