import argparse
import asyncio
import logging
import os
import time

import mingus.core.scales as scales
import mingus.midi.fluidsynth as fluidsynth
from mingus.containers import Note

from tabletop_py.flic.client import (
    ButtonConnectionChannel,
    ClickType,
    FlicClient,
)

logger = logging.getLogger(__name__)

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


bd_addr_to_note = {}


class PianoButtonConnectionChannel(ButtonConnectionChannel):  # type: ignore
    def __init__(self, *args, note: Note, **kwargs):
        super().__init__(*args, **kwargs)
        self.note = note

    def on_button_event(self, click_type, was_queued, time_diff, event_time):
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
    if not fluidsynth.init(soundfont_path, driver=driver):
        raise RuntimeError("Failed to initialize fluidsynth")

    scale_cls = getattr(scales, scale)
    scale = scale_cls(key).ascending()[:-1]
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
    loop = asyncio.get_event_loop()
    _, client = await loop.create_connection(
        lambda: FlicClient(loop=loop), host, port
    )
    init_piano(soundfont, key, octave, scale, driver)

    try:
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


def test_note():
    soundfont_path = os.path.join(
        os.environ["TABLETOP_DIR"], "soundfonts", "moog.sf2"
    )
    if not os.path.exists(soundfont_path):
        raise FileNotFoundError(f"Soundfont {soundfont_path} not found")
    fluidsynth.init(soundfont_path, driver="pulseaudio")
    fluidsynth.set_instrument(0, 62)
    note = Note("C", 4, velocity=127, channel=0)
    fluidsynth.play_Note(note)
    time.sleep(1)


if __name__ == "__main__":
    main()


# sudo apt install fluidsynth
# pip install mingus
