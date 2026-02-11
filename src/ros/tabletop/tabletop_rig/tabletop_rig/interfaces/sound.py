"""Interface for audio synthesis using FluidSynth.

This module provides an interface for playing sounds during experiments using
the FluidSynth software synthesizer. It supports playing MIDI notes with
configurable soundfonts and instruments.

FluidSynth is a real-time software synthesizer that uses SoundFont (.sf2)
files to generate audio, commonly used for auditory feedback in experiments.
"""

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
    """Interface for playing sounds via FluidSynth.

    Provides methods for starting, stopping, and playing notes with
    configurable duration. The interface can be disabled via configuration
    for silent operation during debugging.

    Attributes:
        enabled: Whether sound playback is enabled.
        _default_note: The default note to play when none is specified.
        _default_duration: The default duration for note playback in seconds.
    """

    def __init__(self, node: BaseNode) -> None:
        """Initialize the sound interface.

        Loads configuration from the node's parameters, initializes FluidSynth
        with the configured soundfont, and sets up the default note and
        instrument.

        Args:
            node: Parent ROS2 node containing sound configuration parameters.

        Raises:
            FileNotFoundError: If the configured soundfont file doesn't exist.
            RuntimeError: If FluidSynth fails to initialize.
            ValueError: If default_duration is not a positive number.
        """
        super().__init__("sound_interface", node)

        config: dict[str, Any] = self.node.param("sound")

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

    def init_sound(self) -> None:
        """Initialize sound system (placeholder for future use)."""

    def start_note(self, note: Optional[Note] = None) -> None:
        """Start playing a note continuously.

        The note plays until explicitly stopped with stop_note().

        Args:
            note: The note to play. If None, uses the default note.
        """
        if self.enabled:
            if note is None:
                note = self._default_note
            fluidsynth.play_Note(note)

    def stop_note(self, note: Optional[Note] = None) -> None:
        """Stop a currently playing note.

        Args:
            note: The note to stop. If None, stops the default note.
        """
        if self.enabled:
            if note is None:
                note = self._default_note
            fluidsynth.stop_Note(note)

    def stop_everything(self) -> None:
        """Stop all currently playing notes.

        Useful for cleanup or when aborting an experiment.
        """
        if self.enabled:
            fluidsynth.stop_everything()

    async def play(
        self,
        note: Optional[Note | Mapping[str, Any]] = None,
        duration: Optional[float] = None,
    ) -> None:
        """Play a note for a specified duration asynchronously.

        Starts the note, waits for the duration using async sleep, then
        stops the note. This allows other async tasks to run during playback.

        Args:
            note: Note to play. Can be a mingus Note object or a dict with
                Note constructor arguments. If None, uses the default note.
            duration: How long to play in seconds. If None, uses the default
                duration from configuration.
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

    def destroy_interface(self):
        self.log("Destroying SoundInterface")
        if fluidsynth.initialized:
            fluidsynth.stop_everything()
        super().destroy_interface()
