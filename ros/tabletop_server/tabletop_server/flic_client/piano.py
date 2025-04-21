import argparse
import asyncio
import os
import subprocess
import time

import mingus.core.scales as scales
import mingus.midi.fluidsynth as fluidsynth
from client_aio import (
    ButtonConnectionChannel,
    ClickType,
    FlicClient,
)
from mingus.containers import Note

bd_addr_ordered = [
    "80:e4:da:7c:a6:0b",
    "80:e4:da:7e:2a:eb",
    "80:e4:da:7e:2b:31",
    "80:e4:da:7e:5c:af",
    "80:e4:da:7e:64:8c",
    "80:e4:da:7e:64:96",
    "80:e4:da:7e:64:a0",
    "80:e4:da:7e:5c:8e",
]


bd_addr_to_note = {}


def on_button_up_or_down(self, click_type, was_queued, time_diff):
    self._on_button("Button up or down", click_type, was_queued, time_diff)
    note = bd_addr_to_note[self.bd_addr]
    if click_type == ClickType.ButtonDown:
        fluidsynth.play_Note(note)


ButtonConnectionChannel.on_button_up_or_down = on_button_up_or_down


def init_piano(soundfont: str, key: str, octave: int, scale: str):
    if not fluidsynth.init(soundfont, driver="pulseaudio"):
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
    fluidsynth_process = subprocess.Popen(["fluidsynth", "-a", driver, "-i"])

    try:
        await client.connect_existing_buttons()
        print(
            f"Waiting for {len(bd_addr_to_note) - client.num_buttons} buttons"
        )
        while client.num_buttons < len(bd_addr_to_note):
            await asyncio.sleep(1)
        print(f"Connected to {client.num_buttons} buttons")

        init_piano(soundfont, key, octave, scale)

        print("Spinning")
        await client.wait_for_closed()
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        fluidsynth_process.terminate()
        await client.close()


def main():
    dir_path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "soundfonts"
    )

    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="172.17.0.1")
    parser.add_argument("--port", type=int, default=5551)
    parser.add_argument("--soundfont", type=str, default="lyzen.sf2")
    parser.add_argument("--key", type=str, default="C")
    parser.add_argument("--octave", type=int, default=4)
    parser.add_argument("--scale", type=str, default="HarmonicMinor")
    parser.add_argument("--driver", type=str, default="pulseaudio")
    args = parser.parse_args()

    soundfont_path = os.path.join(dir_path, args.soundfont)

    asyncio.run(
        run(
            host=args.host,
            port=args.port,
            soundfont=soundfont_path,
            driver=args.driver,
            key=args.key,
            octave=args.octave,
            scale=args.scale,
        )
    )


def test_note():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    fluidsynth.init(os.path.join(dir_path, "lyzen.sf2"), driver="pulseaudio")
    time.sleep(1)
    note = Note("C", 4, velocity=127)
    fluidsynth.play_Note(note)
    time.sleep(1)


if __name__ == "__main__":
    # test_note()
    main()


# sudo apt install fluidsynth
# pip install mingus
