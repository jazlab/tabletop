import math
import os
import queue
import threading
import time
from datetime import datetime

import cv2
import numpy as np
import PySpin

# thumbnail size
THUMB_W, THUMB_H = 640, 480
# save dirs (to be set per session-run)
SAVE_DIR_IMG = None
SAVE_DIR_VID = None

# thread-safe queue for CLI commands
command_queue = queue.Queue()
# global recording controls
dynamic_recording_start = None
record_threads = {}  # idx -> Thread
decoded_buffers = {}  # idx -> list of frames
record_flags = {}  # idx -> Event
fps_list = []  # locked FPS per camera


def input_thread():
    """
    Runs in background. Reads lines from stdin and pushes them to command_queue.
    """
    while True:
        try:
            cmd = input("cmd> ").strip()
        except EOFError:
            cmd = "exit"
        command_queue.put(cmd)
        if cmd.lower() in ("e", "q", "exit", "quit"):
            break


def ensure_dir(d):
    if not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)
        print(f"Created directory: {d}/")


def print_camera_info(cam, idx, fps):
    tl = cam.GetTLDeviceNodeMap()
    vendor = PySpin.CStringPtr(tl.GetNode("DeviceVendorName")).GetValue()
    model = PySpin.CStringPtr(tl.GetNode("DeviceModelName")).GetValue()
    serial = PySpin.CStringPtr(tl.GetNode("DeviceSerialNumber")).GetValue()
    nl = cam.GetNodeMap()
    w = PySpin.CIntegerPtr(nl.GetNode("Width")).GetValue()
    h = PySpin.CIntegerPtr(nl.GetNode("Height")).GetValue()
    pf = PySpin.CEnumEntryPtr(
        PySpin.CEnumerationPtr(nl.GetNode("PixelFormat")).GetCurrentEntry()
    ).GetSymbolic()
    print(
        f"Cam{idx}: {vendor} {model} (S/N {serial}) — {w}x{h}, {pf}, locked to {fps:.1f} FPS"
    )


def save_frame(frame, cam_idx):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = f"cam_{cam_idx}_{ts}.png"
    path = os.path.join(SAVE_DIR_IMG, fname)
    cv2.imwrite(path, frame)
    print(f"Saved image: {path}")


def record_worker(idx, get_frame, writer, flag, fps):
    """
    Worker thread to write frames from get_frame() at fixed fps until flag is set.
    """
    interval = 1.0 / fps
    while not flag.is_set():
        frame = get_frame(idx)
        writer.write(frame)
        time.sleep(interval)
    writer.release()
    print(f"Stopped recording Cam{idx}")


def start_recording(indices, last_thumbs):
    """
    Countdown then spawn a thread per camera index to record using locked FPS.
    """
    global dynamic_recording_start, record_threads, record_flags, fps_list
    print("Recording starts in 3...")
    time.sleep(1)
    print("2...")
    time.sleep(1)
    print("1...")
    time.sleep(1)
    print("Recording!")
    dynamic_recording_start = time.time()

    for idx in indices:
        if idx in record_threads:
            continue  # already recording
        fps = fps_list[idx]
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"cam_{idx}_{ts}.avi"
        path = os.path.join(SAVE_DIR_VID, fname)
        writer = cv2.VideoWriter(
            path, cv2.VideoWriter_fourcc(*"MJPG"), fps, (THUMB_W, THUMB_H)
        )
        flag = threading.Event()
        record_flags[idx] = flag
        thread = threading.Thread(
            target=record_worker,
            args=(idx, lambda i: last_thumbs[i], writer, flag, fps),
            daemon=True,
        )
        record_threads[idx] = thread
        thread.start()
        print(f"Started recording Cam{idx} -> {path}")


def stop_recording():
    """
    Signal all record threads to stop, wait for them, and report elapsed.
    """
    global dynamic_recording_start, record_threads, record_flags
    if not record_threads:
        print("No active recordings to stop.")
        return
    # signal threads
    for flag in record_flags.values():
        flag.set()
    # wait for threads
    for th in record_threads.values():
        th.join()
    record_threads.clear()
    record_flags.clear()
    # report elapsed
    elapsed = (
        int(time.time() - dynamic_recording_start)
        if dynamic_recording_start
        else 0
    )
    hh, rem = divmod(elapsed, 3600)
    mm, ss = divmod(rem, 60)
    print(f"Total recording time: {hh:02d}:{mm:02d}:{ss:02d}")
    dynamic_recording_start = None


def main():
    # session/run prompt
    session = input("Enter session number: ").strip()
    run = input("Enter run number: ").strip()
    date_s = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_dir = os.path.join("data", f"session_{session}_run_{run}_{date_s}")

    global SAVE_DIR_IMG, SAVE_DIR_VID, fps_list
    SAVE_DIR_IMG = os.path.join(base_dir, "img")
    SAVE_DIR_VID = os.path.join(base_dir, "videos")

    ensure_dir(SAVE_DIR_IMG)
    ensure_dir(SAVE_DIR_VID)

    # start CLI thread
    th = threading.Thread(target=input_thread, daemon=True)
    th.start()

    # init Spinnaker
    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()
    n = cam_list.GetSize()
    print(f"Detected {n} camera(s).\n")
    if n == 0:
        cam_list.Clear()
        system.ReleaseInstance()
        return

    cams = []
    fps_list = []
    for i in range(n):
        cam = cam_list.GetByIndex(i)
        cam.Init()
        # lock FPS to maximum
        nl = cam.GetNodeMap()
        fr_enable = PySpin.CBooleanPtr(
            nl.GetNode("AcquisitionFrameRateEnable")
        )
        if not fr_enable.GetValue():
            fr_enable.SetValue(True)
        fr_node = PySpin.CFloatPtr(nl.GetNode("AcquisitionFrameRate"))
        max_fps = fr_node.GetMax()
        fr_node.SetValue(max_fps)
        fps_list.append(max_fps)

        print_camera_info(cam, i, max_fps)
        cam.BeginAcquisition()
        cams.append(cam)

    # prepare display window
    cv2.namedWindow("AllCams", cv2.WINDOW_NORMAL)

    last_thumbs = [np.zeros((THUMB_H, THUMB_W, 3), np.uint8) for _ in range(n)]

    print("\nStream started.")
    print("Commands:")
    print("  c [idx|all]         capture image")
    print("  v [idx,idx|all]     start recording")
    print("  s                   stop recordings")
    print("  e/q                 exit\n")

    try:
        while True:
            # capture & display thumbnails
            thumbs = []
            for idx, cam in enumerate(cams):
                img = cam.GetNextImage()
                if img.IsIncomplete():
                    img.Release()
                    thumbs.append(last_thumbs[idx])
                    continue
                raw = img.GetNDArray()
                img.Release()
                frame = (
                    cv2.cvtColor(raw, cv2.COLOR_BAYER_RG2BGR)
                    if raw.ndim == 2
                    else raw
                )
                thumb = cv2.resize(frame, (THUMB_W, THUMB_H))
                thumbs.append(thumb)
                last_thumbs[idx] = thumb

            # arrange thumbnails in a grid if more than 3 cameras
            n_thumbs = len(thumbs)
            # determine grid size
            cols = (
                n_thumbs
                if n_thumbs <= 3
                else int(math.ceil(math.sqrt(n_thumbs)))
            )
            rows = int(math.ceil(n_thumbs / cols))
            # pad with blank frames if needed
            pad = cols * rows - n_thumbs
            if pad > 0:
                thumbs += [
                    np.zeros((THUMB_H, THUMB_W, 3), dtype=np.uint8)
                ] * pad
            # build grid rows
            grid_rows = []
            for r in range(rows):
                row_frames = thumbs[r * cols : (r + 1) * cols]
                grid_rows.append(cv2.hconcat(row_frames))
            grid = cv2.vconcat(grid_rows)

            # resize window to fit grid
            cv2.resizeWindow("AllCams", THUMB_W * cols, THUMB_H * rows)
            cv2.imshow("AllCams", grid)
            cv2.waitKey(1)

            # show stopwatch if recording
            if dynamic_recording_start and record_threads:
                elapsed = int(time.time() - dynamic_recording_start)
                hh, rem = divmod(elapsed, 3600)
                mm, rem = divmod(rem, 60)
                ss = int(rem)
                ms = int((rem - ss) * 1000)
                print(
                    f"\rRecording: {int(hh):02d}:{int(mm):02d}:{ss:02d}.{ms:03d}",
                    end="",
                    flush=True,
                )

            # process commands
            try:
                cmd = command_queue.get_nowait()
            except queue.Empty:
                cmd = None

            if not cmd:
                continue
            parts = cmd.lower().split()
            if parts[0] in ("e", "q", "exit", "quit"):
                break
            if parts[0] == "c":
                if len(parts) != 2:
                    print("Usage: c [idx|all]")
                elif parts[1] == "all":
                    for ci in range(n):
                        save_frame(last_thumbs[ci], ci)
                else:
                    try:
                        ci = int(parts[1])
                        if 0 <= ci < n:
                            save_frame(last_thumbs[ci], ci)
                        else:
                            print(f"Invalid index {ci}")
                    except ValueError:
                        print(f"Invalid argument {parts[1]}")
            elif parts[0] == "v":
                if len(parts) != 2:
                    print("Usage: v [idx,idx|all]")
                else:
                    if parts[1] == "all":
                        idxs = list(range(n))
                    else:
                        try:
                            idxs = [int(x) for x in parts[1].split(",")]
                        except ValueError:
                            print(f"Invalid list {parts[1]}")
                            continue
                    valid = [i for i in idxs if 0 <= i < n]
                    if not valid:
                        print("No valid camera indices.")
                        continue
                    start_recording(valid, last_thumbs)
            elif parts[0] == "s":
                stop_recording()
            else:
                print(f"Unknown command: {cmd}")

    finally:
        cv2.destroyAllWindows()
        stop_recording()
        for cam in cams:
            try:
                cam.EndAcquisition()
            except:
                pass
            try:
                cam.DeInit()
            except:
                pass
        try:
            cam_list.Clear()
        except:
            pass
        del cams, cam_list
        try:
            system.ReleaseInstance()
        except:
            pass
        print("Stream stopped.")


if __name__ == "__main__":
    main()
