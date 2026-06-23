"""Musical keyboard interface using Flic buttons.

This module implements a piano-like instrument using Flic Bluetooth buttons
as keys. Each button is mapped to a musical note in a configurable scale,
with sound synthesis provided by FluidSynth.

The module uses mingus for music theory (scales, notes) and fluidsynth
for MIDI synthesis. Buttons are mapped sequentially to notes in the
chosen scale, with automatic octave progression.

Classes:
    PianoButtonConnectionChannel: Button channel that plays notes on press.

Functions:
    init_piano: Initialize FluidSynth and map buttons to notes.
    run: Main async loop connecting buttons and playing notes.
    main: CLI entry point.
    test_note: Simple test function to verify audio setup.

Example:
    # Run from command line
    python -m tabletop_py.flic.piano --key C --scale HarmonicMinor --octave 4

Dependencies:
    - FluidSynth (apt install fluidsynth)
    - mingus (pip install mingus)
    - A SoundFont file (.sf2)
"""

import argparse
import asyncio
import logging

import mingus.core.scales as scales
import mingus.midi.fluidsynth as fluidsynth
from mingus.containers import Note

from tabletop_py.flic.client import (
    ButtonConnectionChannel,
    ClickType,
    FlicClient,
)

logger = logging.getLogger(__name__)

#: Bluetooth addresses of buttons in order (maps to scale notes)
bd_addr_ordered = [
    "90:88:a9:50:5f:b6",
    "90:88:a9:50:5f:92",
    "90:88:a9:50:63:08",
    "90:88:a9:50:61:4e",
    "90:88:a9:50:60:9f",
    "90:88:a9:50:65:ff",
    "90:88:a9:50:60:09",
    "90:88:a9:50:7c:7a",
    "90:88:a9:50:7c:b1",
    "90:88:a9:50:65:fb",
]

#: Mapping from button address to mingus Note object
bd_addr_to_note = {}


class PianoButtonConnectionChannel(ButtonConnectionChannel):  # type: ignore
    """Button connection channel that plays a musical note on press.

    Extends ButtonConnectionChannel to trigger FluidSynth note playback
    when the button is pressed (ButtonDown event).

    Attributes:
        note: The mingus Note object to play when button is pressed.
    """

    def __init__(self, *args, note: Note, **kwargs):
        """Initialize the piano button channel.

        Args:
            *args: Positional arguments passed to ButtonConnectionChannel.
            note: The mingus Note to play on button press.
            **kwargs: Keyword arguments passed to ButtonConnectionChannel.
        """
        super().__init__(*args, **kwargs)
        self.note = note

    def on_button_event(self, click_type, was_queued, time_diff, event_time):
        """Handle button events by playing the assigned note.

        Plays the note via FluidSynth when a ButtonDown event is received.

        Args:
            click_type: Type of button event (ButtonDown, ButtonUp, etc.)
            was_queued: Whether this event was queued while disconnected.
            time_diff: Time difference from button press to event receipt.
            event_time: Timestamp of the event.
        """
        logger.info(
            f"Button event: {click_type} | "
            f"addr: {self.bd_addr}, "
            f"was_queued: {was_queued}, "
            f"time_diff: {time_diff}, "
            f"time: {event_time}"
        )
        if click_type == ClickType.ButtonDown:
            fluidsynth.play_Note(self.note)


def init_piano(
    soundfont_path: str, key: str, octave: int, scale: str, driver: str
):
    """Initialize FluidSynth and create button-to-note mapping.

    Sets up the FluidSynth synthesizer with the specified soundfont and
    audio driver, then maps each button address to a note in the chosen
    scale. Octaves automatically increment when passing through C.

    Args:
        soundfont_path: Path to the SoundFont (.sf2) file.
        key: Root key of the scale (e.g., "C", "G", "F#").
        octave: Starting octave number.
        scale: Name of the mingus scale class (e.g., "HarmonicMinor", "Major").
        driver: FluidSynth audio driver (e.g., "pulseaudio", "alsa").

    Raises:
        RuntimeError: If FluidSynth initialization fails.
    """
    if not fluidsynth.init(soundfont_path, driver=driver):
        raise RuntimeError("Failed to initialize fluidsynth")

    # Get the scale notes (ascending, excluding octave repeat)
    scale_cls = getattr(scales, scale)
    scale = scale_cls(key).ascending()[:-1]

    # Map each button to a note, incrementing octave at each C
    i = 0
    for i, bd_addr in enumerate(bd_addr_ordered):
        if "C" in scale[i % len(scale)] and i > 0:
            octave += 1
        bd_addr_to_note[bd_addr] = Note(scale[i % len(scale)], octave)


async def run(
    host: str,
    port: int,
    soundfont: str,
    driver: str,
    key: str,
    octave: int,
    scale: str,
):
    """Run the piano application asynchronously.

    Connects to the Flic daemon, initializes the piano sound system,
    and listens for button events. Runs until the connection closes
    or an error occurs.

    Args:
        host: Flic daemon host address.
        port: Flic daemon port number.
        soundfont: Path to the SoundFont file.
        driver: FluidSynth audio driver name.
        key: Root key of the musical scale.
        octave: Starting octave number.
        scale: Name of the musical scale.
    """
    loop = asyncio.get_event_loop()
    _, client = await loop.create_connection(
        lambda: FlicClient(loop=loop), host, port
    )
    init_piano(soundfont, key, octave, scale, driver)

    try:
        # Run button listener and monitor for connection close
        coro_task = asyncio.create_task(client.spin_listen(period=20))
        closed_task = asyncio.create_task(client.wait_for_closed())

        done, _ = await asyncio.wait(
            [coro_task, closed_task],
            return_when=asyncio.FIRST_COMPLETED,
        )
        for task in done:
            task.result()
    finally:
        client.close()


def main():
    """Command-line entry point for the Flic piano application.

    Parses command-line arguments and runs the piano application.

    Command-line Arguments:
        --host: Flic daemon host (default: 172.17.0.1)
        --port: Flic daemon port (default: 5551)
        -s/--soundfont: SoundFont file path (default: lyzen.sf2)
        -k/--key: Musical key (default: C)
        -o/--octave: Starting octave (default: 4)
        --scale: Scale type (default: HarmonicMinor)
        --driver: Audio driver (default: pulseaudio)
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="172.17.0.1")
    parser.add_argument("--port", type=int, default=5551)
    parser.add_argument("-s", "--soundfont", type=str, default="lyzen.sf2")
    parser.add_argument("-k", "--key", type=str, default="C")
    parser.add_argument("-o", "--octave", type=int, default=4)
    parser.add_argument("--scale", type=str, default="HarmonicMinor")
    parser.add_argument("--driver", type=str, default="pulseaudio")
    args = parser.parse_args()

    asyncio.run(run(**vars(args)))


if __name__ == "__main__":
    main()


# sudo apt install fluidsynth
# pip install mingus
