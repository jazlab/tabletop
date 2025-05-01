import concurrent.futures
import logging
import os
import subprocess
import threading
import time
import traceback
from collections import deque
from datetime import datetime
from threading import Lock
from typing import Any, Optional

import numpy as np
import pylink
import rclpy
from pylink import EyeLink as EyeLinkTracker
from pylink.constants import MISSING_DATA
from pylink.tracker import Sample, SampleData
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from tabletop_msgs.srv import GetSmoothPursuit

from tabletop_server.nodes.base import BaseNode

DEFAULT_LOG_SEVERITY = "INFO"
TRACEBACK = False
NO_EYE = -1
LEFT_EYE = 0
RIGHT_EYE = 1
BINOCULAR = 2


class Eyelink(BaseNode):
    default_params = BaseNode.default_params | {
        "tracker_address": "100.1.1.1",
        "do_tracker_setup": True,
        "wait_for_data_timeout": 1.0,
        "log_periodic_interval": 0.5,
        "sample_rate": 1000,
        "smooth_pursuit_window": 0.5,
        "smooth_pursuit_default_threshold": 100.0,
        "smooth_pursuit_min_samples": 100,
        "conversion_script_path": "/root/ws/src/tabletop/scripts/eyelink_convert.sh",
        "results_dir": "/root/ws/src/tabletop/results/eyelink",
        "file_event_flags": "LEFT,RIGHT,FIXATION,SACCADE,BLINK,MESSAGE,BUTTON,INPUT",
        "file_sample_flags": "LEFT,RIGHT,RAW,AREA,STATUS,INPUT",
        "link_event_flags": "LEFT,RIGHT,FIXATION,SACCADE,BLINK,BUTTON,FIXUPDATE,INPUT",
        "link_sample_flags": "LEFT,RIGHT,RAW,AREA,STATUS,INPUT",
    }

    ###########################################################################
    # Initialization
    ###########################################################################

    def __init__(self):
        super().__init__("eyelink")

        self.tracker = EyeLinkTracker(
            self.get_parameter_wrapper("tracker_address")
        )
        pylink.endRealTimeMode()

        self.setup_ros()
        self.setup_sample_queue()

        self.destroyed = False

    def setup_ros(self):
        """Setup the ROS node.

        This function will setup the ROS node. It will create a timer to log
        the eyelink status and a series of services to control the tracker.
        """
        self.log_periodic_timer = self.create_timer(
            self.get_parameter_wrapper("log_periodic_interval"),
            self.log_periodic_callback,
        )
        self.get_smooth_pursuit_service = self.create_service(
            GetSmoothPursuit,
            "eyelink/get_smooth_pursuit",
            self.get_smooth_pursuit_callback,
        )
        self.eyelink_start_recording_service = self.create_service(
            Trigger,
            "eyelink/start_recording",
            self.start_recording_callback,
        )
        self.eyelink_stop_recording_service = self.create_service(
            Trigger,
            "eyelink/stop_recording",
            self.stop_recording_callback,
        )
        self.eyelink_open_data_file_service = self.create_service(
            Trigger,
            "eyelink/open_data_file",
            self.open_data_file_callback,
        )
        self.eyelink_close_data_file_service = self.create_service(
            Trigger,
            "eyelink/close_data_file",
            self.close_data_file_callback,
        )

    def setup_sample_queue(self):
        """Setup the sample queue.

        This function will setup the sample queue. The sample queue is used
        to store a window of the most recent samples from the tracker.
        """
        sample_rate = self.get_parameter_wrapper("sample_rate")
        smooth_pursuit_window = self.get_parameter_wrapper(
            "smooth_pursuit_window"
        )
        self.sample_queue: deque[Sample] = deque(
            maxlen=int(sample_rate * smooth_pursuit_window)
        )
        self.sample_queue_lock = Lock()
        self.stop_get_samples_event = threading.Event()
        self.tpe = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.get_samples_future = self.tpe.submit(
            self.get_samples, stop_event=self.stop_get_samples_event
        )
        self.get_samples_future.add_done_callback(
            lambda _: self.executor.shutdown()  # type: ignore
        )

    ###########################################################################
    # Tracker utilities
    ###########################################################################

    def eyelink_pc_setup(self):
        """Start tracker setup on the Eyelink PC.

        This function will start the tracker setup process on the Eyelink PC.
        It will then wait for the user to press the "ESC" key (on the Eyelink
        PC) to end the setup.

        Note: This function will not respond to a keyboard interrupt (e.g.
        Ctrl+C) from this machine. Make sure to press the "ESC" key to end
        the setup.
        """
        try:
            self.tracker.doTrackerSetup()
        except RuntimeError as e:
            self.log(f"Error doing tracker setup: {e}", severity="WARN")
            self.tracker.exitCalibration()

    def open_data_file(self):
        """Opens a data file on the EyeLink PC.

        This function will open a data file on the EyeLink PC.
        This file will be named "last.edf" and will store the data from the
        current recording session.
        """
        edf_file_name = "last.edf"
        self.tracker.openDataFile(edf_file_name)

        preamble_text = "RECORDED BY EyeLink ROS Node"
        self.tracker.sendCommand(f"add_file_preamble_text '{preamble_text}'")

        self.tracker.setPupilSizeDiameter("YES")

        self.tracker.sendCommand(
            f"file_event_filter = {self.get_parameter_wrapper('file_event_flags')}"
        )
        self.tracker.sendCommand(
            f"file_sample_data = {self.get_parameter_wrapper('file_sample_flags')}"
        )
        self.tracker.sendCommand(
            f"link_event_filter = {self.get_parameter_wrapper('link_event_flags')}"
        )
        self.tracker.sendCommand(
            f"link_sample_data = {self.get_parameter_wrapper('link_sample_flags')}"
        )

    def close_data_file(self):
        """Closes the data file and transfers it to the local machine.

        This function will close the data file on the EyeLink PC and transfer
        it to the local machine. It will then convert the EDF file to ASC
        format.
        """
        self.tracker.setOfflineMode()
        self.tracker.closeDataFile()

        results_dir = self.get_parameter_wrapper("results_dir")
        os.makedirs(results_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M")
        local_file_name = os.path.join(results_dir, f"{timestamp}.edf")
        try:
            self.tracker.receiveDataFile("last.edf", local_file_name)
        except RuntimeError as e:
            self.log(f"Error receiving data file: {e}", severity="ERROR")
            raise RuntimeError("Error receiving data file") from e

        self.convert_edf_to_asc()

    def start_recording(self):
        """Start the tracker recording."""
        self.tracker.setOfflineMode()
        self.tracker.startRecording(1, 0, 1, 0)
        pylink.endRealTimeMode()
        self.tracker.sendMessage("SYNCTIME")

        with self.sample_queue_lock:
            self.sample_queue.clear()

    def stop_recording(self):
        """Stop the recording."""
        self.tracker.stopRecording()

    def convert_edf_to_asc(self):
        """Convert the EDF file to ASC format using the eyelink_convert.sh script."""
        try:
            script_path = self.get_parameter_wrapper("conversion_script_path")
            results_dir = self.get_parameter_wrapper("results_dir")
            subprocess.run([script_path, "-o", results_dir], check=True)
            self.log("Successfully converted EDF to ASC")
        except subprocess.CalledProcessError as e:
            self.log(f"Error converting EDF to ASC: {e}", severity="ERROR")
            raise RuntimeError("Error converting EDF to ASC") from e

    def eyeAvailable(self) -> int:
        """Get the eye available from the tracker.

        This function will return one of the following values:
        - LEFT_EYE: The left eye is available.
        - RIGHT_EYE: The right eye is available.
        - BINOCULAR: Both eyes are available.
        - NO_EYE: No eye is available.
        """
        return self.tracker.eyeAvailable()

    def left_eye_available(self) -> bool:
        """Check if the left eye is available."""
        return self.eyeAvailable() in [LEFT_EYE, BINOCULAR]

    def right_eye_available(self) -> bool:
        """Check if the right eye is available."""
        return self.eyeAvailable() in [RIGHT_EYE, BINOCULAR]

    ###########################################################################
    # Sample retrieval
    ###########################################################################

    def get_samples(self, stop_event: threading.Event):
        """Get samples from the tracker.

        This function will loop indefinitely, getting samples from the
        tracker and adding them to the sample queue. This function is
        thread-safe and should be run in a separate thread (or you'll not
        be able to do anything else).

        Args:
            stop_event: A threading event that can be used to stop the thread
            from another thread.
        """
        try:
            wait_for_data_timeout_ms = int(
                self.get_parameter_wrapper("wait_for_data_timeout") * 1000
            )
            while not stop_event.is_set():
                if not self.tracker.isConnected():
                    self.log("Tracker is not connected", severity="WARN")
                    time.sleep(1)
                    continue
                try:
                    self.tracker.waitForData(wait_for_data_timeout_ms, 1, 1)
                except RuntimeError as e:
                    self.log(
                        f"No data from tracker with error: {e}",
                        severity="WARN",
                    )
                    continue

                test_sample: Sample | None = self.tracker.getSample()
                if test_sample is not None:
                    sample: Any | None = self.tracker.getFloatData()
                    assert sample is not None and isinstance(sample, Sample)
                    with self.sample_queue_lock:
                        self.sample_queue.append(sample)
        except Exception as e:
            self.log(
                f"Error in get_samples: type={type(e)}, value={e}",
                severity="ERROR",
            )
            if TRACEBACK:
                traceback_str = traceback.format_exc()
                self.log(f"Traceback: {traceback_str}", severity="ERROR")
            raise e

    def get_last_samples(self) -> list[Sample]:
        """Get the latest samples from the eyelink tracker.

        This function will return a copy of the contents of the sample queue
        as a list.
        """
        with self.sample_queue_lock:
            return list(self.sample_queue)

    def get_valid_eye_data(
        self, sample: Sample
    ) -> (
        tuple[
            int,
            tuple[float, float, float] | None,
            tuple[float, float, float] | None,
        ]
        | None
    ):
        """Get the valid eye data from the sample.

        This function will return the timestamp and the eye data for the left
        and right eyes, if available.

        Args:
            sample: The sample to get the eye data from.

        Returns:
            A tuple containing the timestamp and the eye data for the left
            and right eyes, if available and valid. If only one eye is
            available/valid, the other eye will be None. If no eye data is
            available/valid, the function will return None.
        """
        timestamp: int = sample.getTime()
        left_sample: SampleData | None = sample.getLeftEye()
        right_sample: SampleData | None = sample.getRightEye()

        eye_used = self.eyeAvailable()
        if eye_used == LEFT_EYE:
            assert left_sample is not None
            assert right_sample is None
        elif eye_used == RIGHT_EYE:
            assert left_sample is None
            assert right_sample is not None
        elif eye_used == BINOCULAR:
            assert left_sample is not None
            assert right_sample is not None
        elif eye_used == NO_EYE:
            assert left_sample is None
            assert right_sample is None
            self.log("No eye available", severity="WARN")
            return None
        else:
            assert False, f"Unknown eye available value: {eye_used}"

        if left_sample is None:
            left_data = None
        else:
            left_x, left_y = left_sample.getRawPupil()
            left_diameter = left_sample.getPupilSize()
            if MISSING_DATA in (left_x, left_y, left_diameter):
                left_data = None
            else:
                left_data = (left_x, left_y, left_diameter)

        if right_sample is None:
            right_data = None
        else:
            right_x, right_y = right_sample.getRawPupil()
            right_diameter = right_sample.getPupilSize()
            if MISSING_DATA in (right_x, right_y, right_diameter):
                right_data = None
            else:
                right_data = (right_x, right_y, right_diameter)

        if left_data is None and right_data is None:
            return None

        return timestamp, left_data, right_data

    ###########################################################################
    # Smooth pursuit
    ###########################################################################

    def get_smooth_pursuit(self, threshold: Optional[float] = None) -> bool:
        """Check if the subject is smoothly pursuing.

        This function will check if the subject is smoothly pursuing by
        checking if the speed of the left and right eyes is below a threshold
        (the eye can only move smoothly if it is following a smoothly moving
        object, so we check if the speed remains below a threshold).
        """
        samples = self.get_last_samples()
        if len(samples) == 0:
            raise RuntimeError("No samples in queue")

        # Track duration of smooth pursuit extraction, for logging purposes
        start_time = time.time()

        # Extract eye data from samples
        timestamps = []
        left_positions = []
        right_positions = []
        for sample in samples:
            if (eye_data := self.get_valid_eye_data(sample)) is None:
                continue
            timestamp, left_data, right_data = eye_data

            timestamps.append(timestamp)
            if self.left_eye_available():
                assert left_data is not None
                left_positions.append(left_data[:2])
            else:
                left_positions.append((MISSING_DATA, MISSING_DATA))
            if self.right_eye_available():
                assert right_data is not None
                right_positions.append(right_data[:2])
            else:
                right_positions.append((MISSING_DATA, MISSING_DATA))

        timestamps = np.array(timestamps)
        left_positions = np.array(left_positions)
        right_positions = np.array(right_positions)

        # Check if the number of samples is the same for all arrays
        assert (
            timestamps.shape[0]
            == left_positions.shape[0]
            == right_positions.shape[0]
        )

        # Check if there are enough samples for meaningful smooth pursuit
        # extraction
        if timestamps.shape[0] < self.get_parameter_wrapper(
            "smooth_pursuit_min_samples"
        ):
            raise RuntimeError("Not enough valid samples")

        assert not np.any(left_positions == MISSING_DATA) or not np.any(
            right_positions == MISSING_DATA
        ), "Missing data should not be present in both eyes"

        # TODO: Add smoothing and/or filtering to the positional data so as
        # to false negatives (e.g. if a spike of noise occurs for a single
        # sample, it is likely not a saccade/break in smooth pursuit and we
        # should ignore it).
        left_speed = np.linalg.norm(
            np.gradient(left_positions, timestamps, axis=0), axis=1
        )
        right_speed = np.linalg.norm(
            np.gradient(right_positions, timestamps, axis=0), axis=1
        )

        # Ensure that smooth pursuit is occuring by checking if the speeds of
        # the left and right eyes are below a threshold
        if threshold is None:
            threshold = self.get_parameter_wrapper(
                "smooth_pursuit_default_threshold"
            )
        is_smoothly_pursuing = np.all(
            np.stack([left_speed, right_speed], axis=1) < threshold
        ).item()

        # Log the smooth pursuit status and statistics about the eye speed data
        if self.get_logger().get_effective_level() <= logging.DEBUG:
            self.log(
                f"Time taken: {time.time() - start_time}", severity="DEBUG"
            )
            if self.left_eye_available():
                left_speed_rounded = left_speed.round(2)
                self.log(
                    f"Left speed: {left_speed_rounded.tolist()}",
                    severity="DEBUG",
                )
                self.log(
                    f"Left speed min: {left_speed_rounded.min()}, max: {left_speed_rounded.max()}, mean: {left_speed_rounded.mean()}",
                    severity="DEBUG",
                )
            if self.right_eye_available():
                right_speed_rounded = right_speed.round(2)
                self.log(
                    f"Right speed: {right_speed_rounded.tolist()}",
                    severity="DEBUG",
                )
                self.log(
                    f"Right speed min: {right_speed_rounded.min()}, max: {right_speed_rounded.max()}, mean: {right_speed_rounded.mean()}",
                    severity="DEBUG",
                )

        return is_smoothly_pursuing

    ###########################################################################
    # ROS callbacks
    ###########################################################################

    def log_periodic_callback(self):
        """Periodically log the smooth pursuit status and the newest valid sample."""
        self.log("Logging samples", severity="DEBUG")

        samples = self.get_last_samples()

        if len(samples) == 0:
            self.log(
                "No samples in queue during periodic log", severity="WARN"
            )
            return

        try:
            is_smoothly_pursuing = self.get_smooth_pursuit()
            self.log(f"Is smoothly pursuing: {is_smoothly_pursuing}")
        except RuntimeError as e:
            self.log(f"Error getting smooth pursuit: {e}", severity="ERROR")

        for sample in reversed(samples):
            if (eye_data := self.get_valid_eye_data(sample)) is None:
                continue
            _, left_data, right_data = eye_data

            if left_data is not None:
                self.log(
                    f"Left eye x: {left_data[0]}, y: {left_data[1]}, diameter: {left_data[2]}"
                )
            if right_data is not None:
                self.log(
                    f"Right eye x: {right_data[0]}, y: {right_data[1]}, diameter: {right_data[2]}"
                )
            return

        self.log("Found no adequate samples", severity="WARN")

    def open_data_file_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Service to open a data file on the EyeLink PC."""
        try:
            self.open_data_file()
        except RuntimeError as e:
            response.success = False
            response.message = f"Error opening data file: {e}"
            self.log(response.message, severity="ERROR")
            return response

        response.success = True
        response.message = "Data file opened"
        return response

    def close_data_file_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Service to close the data file on the EyeLink PC."""
        try:
            self.close_data_file()
        except RuntimeError as e:
            response.success = False
            response.message = f"Error closing data file: {e}"
            self.log(response.message, severity="ERROR")
            return response

        response.success = True
        response.message = (
            "Data file closed, transferred, and converted to ASC"
        )
        return response

    def start_recording_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Start the recording."""
        try:
            self.start_recording()
        except RuntimeError as e:
            response.success = False
            response.message = f"Error starting recording: {e}"
            self.log(response.message, severity="ERROR")
            return response

        response.success = True
        response.message = "Recording started"
        return response

    def stop_recording_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Stop the recording."""
        try:
            self.stop_recording()
        except RuntimeError as e:
            response.success = False
            response.message = f"Error stopping recording: {e}"
            self.log(response.message, severity="ERROR")
            return response

        response.success = True
        response.message = "Recording stopped"
        return response

    def get_smooth_pursuit_callback(
        self,
        request: GetSmoothPursuit.Request,
        response: GetSmoothPursuit.Response,
    ) -> GetSmoothPursuit.Response:
        """Get the smooth pursuit status."""
        try:
            response.is_smoothly_pursuing = self.get_smooth_pursuit()
        except RuntimeError as e:
            response.is_smoothly_pursuing = False
            response.success = False
            response.message = f"Error getting smooth pursuit: {e}"
            self.log(response.message, severity="ERROR")
            return response

        response.success = True
        response.message = f"Subject is {'smoothly' if response.is_smoothly_pursuing else 'not smoothly'} pursuing"
        return response

    ###########################################################################
    # Node lifecycle
    ###########################################################################

    def destroy_node(self):
        """Destroy the node.

        This function will stop the get_samples loop, stop the recording,
        close the data file, and close the tracker.
        """
        print("Stopping get_samples loop")
        self.stop_get_samples_event.set()
        try:
            self.get_samples_future.result(timeout=1.5)
        except Exception as e:
            print(f"Error in get_samples loop: {e}")

        print("Stopping recording")
        try:
            self.stop_recording()
        except Exception as e:
            print(f"Error stopping recording: {e}")

        print("Closing data file")
        try:
            self.close_data_file()
        except Exception as e:
            print(f"Error closing data file: {e}")

        print("Closing tracker")
        self.tracker.close()

        super().destroy_node()
        self.destroyed = True

    def __del__(self):
        if not self.destroyed:
            self.destroy_node()


def main(args=None):
    logging.basicConfig(level=logging.DEBUG)
    rclpy.init(args=args)
    try:
        executor = SingleThreadedExecutor()
        eyelink = Eyelink()
        executor.add_node(eyelink)

        try:
            print("Opening data file")
            eyelink.open_data_file()
            print("Starting recording")
            eyelink.start_recording()
            print("Spinning")
            executor.spin()
        finally:
            eyelink.destroy_node()
            print("Shutting down executor")
            executor.shutdown()
    finally:
        print("Shutting down rclpy")
        rclpy.shutdown()
