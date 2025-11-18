import argparse
import concurrent.futures
import os
import threading
from collections import deque
from enum import Enum
from typing import Any, cast

import debugpy
import numpy as np
import pandas as pd
import rclpy
import rosbag2_py
import torch
import yaml
from geometry_msgs.msg import Point
from mocap4r2_msgs.msg import Marker, Markers
from rclpy.action.server import (
    ActionServer,
    CancelResponse,
    GoalResponse,
    ServerGoalHandle,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.serialization import serialize_message
from rclpy.time import Time
from std_srvs.srv import Trigger
from tabletop_interfaces.action import EyelinkSmoothPursuit
from tabletop_interfaces.msg import Eyelink as EyelinkMsg

from tabletop_py.gaze.edf import edf_to_csv
from tabletop_py.gaze.preprocess import (
    EYELINK_DATA_COLS,
    calculate_eyelink_speed,
    clean_eyelink_data,
    reindex_and_interpolate_eyelink_data,
    smooth_eyelink_data,
)
from tabletop_py.gaze.utils import init_model
from tabletop_rig.exceptions import ROSSleepError
from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.ros import seconds_from_ros_time

PYLINK_AVAILABLE = True
try:
    from pylink import EyeLink as EyeLinkTracker
    from pylink.constants import MISSING_DATA
    from pylink.tracker import Sample, SampleData
except ImportError:
    PYLINK_AVAILABLE = False
    type EyelinkTracker = Any
    type Sample = Any
    type SampleData = Any
    MISSING_DATA = -32768


class DataFileReceiveError(Exception):
    """Error while receiving the data file."""


class DataFileConversionError(Exception):
    """Error while converting the data file."""


class EyeAvailable(Enum):
    NO_EYE = -1
    LEFT_EYE = 0
    RIGHT_EYE = 1
    BINOCULAR = 2


class EyelinkMessageQueue:
    """Thread-safe message queue for Eyelink messages."""

    def __init__(self, maxlen: int):
        self.queue = deque[EyelinkMsg](maxlen=maxlen)
        self.lock = threading.Lock()

    def append(self, msg: EyelinkMsg):
        """Append a message to the queue."""
        with self.lock:
            self.queue.append(msg)

    def to_list(self) -> list[EyelinkMsg]:
        """Get the latest messages from the queue."""
        with self.lock:
            return list(self.queue)

    def clear(self):
        """Clear the queue."""
        with self.lock:
            self.queue.clear()


class Eyelink(BaseNode):
    default_params = BaseNode.default_params | {
        "tracker_address": "192.168.13.30",
        "do_tracker_setup": True,
        "wait_for_data_timeout": 0.1,  # seconds
        "sample_rate": 1000,  # Hz
        "link_sample_data": "LEFT,RIGHT,RAW,AREA,INPUT,STATUS",
        "file_sample_data": "LEFT,RIGHT,RAW,AREA,INPUT,STATUS",
        "file_event_filter": "null",
        "link_event_filter": "null",
        "file_event_data": "null",
        "link_event_data": "null",
        "edf2asc_extra_args": ["-s", "-input", "-nflags", "-y"],
        "session_bag_dir": "null",
        "smooth_pursuit.window": 0.1,  # seconds
        "smooth_pursuit.clean.max_zscore": "null",
        "smooth_pursuit.reindex_and_interpolate.tolerance": 0.003,  # seconds
        "smooth_pursuit.smooth.window": 0.05,  # seconds
        "smooth_pursuit.max_speed": 30000,  # scaled_units/s
        "smooth_pursuit.min_speed": 1000,  # scaled_units/s
        "smooth_pursuit.min_samples": 80,
        "live_gaze_estimation": True,
        "gaze_estimation_config": "$TABLETOP_DIR/config/gaze_estimation.yaml",
        "gaze_estimation_frequency": 100,  # Hz
        "simulate": False,
    }

    ###########################################################################
    # Initialization
    ###########################################################################

    def __init__(self):
        super().__init__("eyelink")

        self.simulate = self.get_parameter_wrapper("simulate")

        if not self.simulate:
            if not PYLINK_AVAILABLE:
                raise RuntimeError(
                    "pylink module not available, make sure the Eyelink and "
                    "Pylink libraries are installed or set simulate to true"
                )
            self.log("Pylink available, connecting to Eyelink machine")
            self.tracker = EyeLinkTracker(  # type: ignore
                self.get_parameter_wrapper("tracker_address")
            )
        else:
            self.log("Simulating eyelink data...")

        # pylink.endRealTimeMode()

        self.init_sample_retrieval()
        self.init_bag_writer()
        self.init_gaze_estimation()
        self.init_ros()
        self.destroyed = False

    def init_sample_retrieval(self):
        """Setup the sample retrieval.

        This function will setup the sample queue and thread pool, as well as
        the stop event and sample retrieval loop future.
        """
        sample_rate = self.get_parameter_wrapper("sample_rate")
        self.smooth_pursuit_window = self.get_parameter_wrapper(
            "smooth_pursuit.window"
        )
        self.message_queue = EyelinkMessageQueue(
            maxlen=int(sample_rate * self.smooth_pursuit_window)
        )
        # self.tpe = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.stop_sample_retrieval_event = threading.Event()
        self.stop_sample_retrieval_event.set()
        self.sample_retrieval_future = concurrent.futures.Future()
        self.sample_retrieval_future.set_result(None)
        self.recording = False

    def init_bag_writer(self):
        """Setup the bag writer.

        This function will setup the bag writer if the session bag directory is
        set.
        """
        self.session_bag_dir = self.get_parameter_wrapper("session_bag_dir")
        if self.session_bag_dir is None:
            self.log(
                "No session bag directory provided, skipping bag writer",
                severity="WARN",
            )
            return

        if not os.path.isdir(self.session_bag_dir):
            raise ValueError(
                f"Session bag directory {self.session_bag_dir} is not a directory"
            )

        bag_dir = os.path.join(self.session_bag_dir, "eyelink")
        self.bag_writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri=bag_dir, storage_id="mcap"
        )
        converter_options = rosbag2_py.ConverterOptions("", "")
        self.bag_writer.open(storage_options, converter_options)
        self.bag_writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=0,
                name="/eyelink/sample",
                type="tabletop_interfaces/msg/Eyelink",
                serialization_format="cdr",
            )
        )

    def init_gaze_estimation(self):
        """Setup the gaze estimation model."""
        path = os.path.expandvars(
            self.get_parameter_wrapper("gaze_estimation_config")
        )
        with open(path, "r") as f:
            self.gaze_estimation_config = yaml.safe_load(f)

        sample_rate = self.get_parameter_wrapper("sample_rate")
        eyelink_freq = self.gaze_estimation_config["eyelink_freq"]
        if sample_rate != eyelink_freq:
            raise ValueError(
                f"Sample rate ({sample_rate}) and gaze estimation eyelink frequency ({eyelink_freq}) must be the same"
            )

        self.preprocess_config = self.gaze_estimation_config["preprocess"]
        self.preprocess_config["clean_eyelink"].update(
            self.get_parameter_wrapper("smooth_pursuit.clean")
        )
        self.preprocess_config["reindex_and_interpolate_eyelink"].update(
            self.get_parameter_wrapper(
                "smooth_pursuit.reindex_and_interpolate"
            )
        )
        self.preprocess_config["smooth_eyelink"].update(
            self.get_parameter_wrapper("smooth_pursuit.smooth")
        )

        if self.get_parameter_wrapper("live_gaze_estimation"):
            self.gaze_estimation_model = init_model(
                **self.gaze_estimation_config["model"]
            )
            self.gaze_estimation_model.eval()

    def init_ros(self):
        """Setup the ROS node.

        This function will setup the ROS node. It will create a timer to log
        the eyelink status and a series of services to control the tracker.
        """
        # Services

        self.eyelink_start_recording_service = self.create_service(
            Trigger,
            "/eyelink/start_recording",
            self.start_recording_callback,
        )
        self.eyelink_stop_recording_service = self.create_service(
            Trigger,
            "/eyelink/stop_recording",
            self.stop_recording_callback,
        )
        self.eyelink_open_data_file_service = self.create_service(
            Trigger,
            "/eyelink/open_data_file",
            self.open_data_file_callback,
        )
        self.eyelink_close_data_file_service = self.create_service(
            Trigger,
            "/eyelink/close_data_file",
            self.close_data_file_callback,
        )

        # Action servers

        self.eyelink_smooth_pursuit_server = ActionServer(
            self,
            EyelinkSmoothPursuit,
            "/eyelink/smooth_pursuit",
            self.smooth_pursuit_callback,
            cancel_callback=self.smooth_pursuit_cancel_callback,
            goal_callback=self.smooth_pursuit_goal_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.goal_callback_lock = threading.Lock()
        self.goal_ongoing = False

        # Publishers

        if hasattr(self, "gaze_estimation_model"):
            self.gaze_estimation_publisher = self.create_publisher(
                Markers,
                "/predicted_markers",
                qos_profile=1000,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
            self.gaze_estimation_timer = self.create_timer(
                1 / self.get_parameter_wrapper("gaze_estimation_frequency"),
                self.gaze_estimation_callback,
                callback_group=MutuallyExclusiveCallbackGroup(),
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
        if self.simulate:
            raise RuntimeError("Simulating eyelink, cannot do tracker setup")

        self.log("Starting tracker setup on the Eyelink PC")
        try:
            self.tracker.doTrackerSetup()
        except RuntimeError as e:
            self.log(f"Error doing tracker setup: {e}", severity="WARN")
            self.tracker.exitCalibration()
        self.log("Eyelink PC tracker setup complete")

    def open_data_file(self):
        """Opens a data file on the EyeLink PC.

        This function will open a data file on the EyeLink PC.
        This file will be named "last.edf" and will store the data from the
        current recording session.
        """
        if self.simulate:
            self.log(
                "Simulating eyelink, skipping data file opening",
                severity="WARN",
            )
            return

        self.log("Opening data file")
        edf_file_name = "last.edf"
        self.tracker.openDataFile(edf_file_name)

        preamble_text = "RECORDED BY EyeLink ROS Node"
        self.tracker.sendCommand(f"add_file_preamble_text '{preamble_text}'")

        self.tracker.setPupilSizeDiameter("YES")

        for key in [
            "file_sample_data",
            "link_sample_data",
            "file_event_filter",
            "link_event_filter",
            "file_event_data",
            "link_event_data",
        ]:
            value = self.get_parameter_wrapper(key)
            if value is not None:
                self.tracker.sendCommand(f"{key} = {value}")

    def start_sample_retrieval(self):
        """Start the sample retrieval loop."""
        self.log("Starting sample retrieval")
        assert self.stop_sample_retrieval_event.is_set()
        assert self.sample_retrieval_future.done(), (
            "Sample retrieval already running, may be in the process of stopping"
        )

        self.message_queue.clear()

        self.stop_sample_retrieval_event.clear()
        # self.sample_retrieval_future = self.tpe.submit(
        #     self.sample_retrieval_loop,
        #     stop_event=self.stop_sample_retrieval_event,
        # )
        if self.executor is None:
            raise RuntimeError("Executor for Eyelink node is not set")
        self.sample_retrieval_future = self.executor.create_task(
            self.sample_retrieval_loop
        )
        self.sample_retrieval_future.add_done_callback(
            lambda _: self.executor.wake()  # type: ignore
        )

    def stop_sample_retrieval(self):
        """Stop the sample retrieval loop."""
        self.log("Stopping sample retrieval")
        self.stop_sample_retrieval_event.set()
        try:
            self.sample_retrieval_future.result()
        except Exception as e:
            self.log(
                f"Error stopping sample retrieval: {type(e).__name__}: {e}",
                severity="ERROR",
            )

    def start_recording(self):
        """Start the tracker recording and sample retrieval."""

        if self.recording:
            self.log("Already recording, skipping start", severity="WARN")
            return

        self.start_sample_retrieval()

        if self.simulate:
            self.log(
                "Simulating eyelink, skipping recording start", severity="WARN"
            )
        else:
            self.log("Starting recording")
            self.tracker.setOfflineMode()
            self.tracker.startRecording(1, 0, 1, 0)
            # pylink.endRealTimeMode()
            self.tracker.sendMessage("SYNCTIME")
            eye_available = self.eye_available()
            if eye_available != EyeAvailable.BINOCULAR:
                self.stop_sample_retrieval()
                raise RuntimeError(
                    f"Only binocular mode is supported, got {eye_available}"
                )

        self.recording = True

    def stop_recording(self):
        """Stop the recording."""
        if not self.recording:
            self.log("Not recording, skipping stop", severity="WARN")
            return

        self.stop_sample_retrieval()

        if self.simulate:
            self.log(
                "Simulating eyelink, skipping recording stop", severity="WARN"
            )
        else:
            self.log("Stopping recording")
            self.tracker.stopRecording()

        self.recording = False

    def close_data_file(self):
        """Closes the data file and transfers it to the local machine.

        This function will stop the recording (if it is running), close the
        data file on the EyeLink PC and transfer it to the local machine. It
        will then convert the EDF file to ASC format.
        """
        self.stop_recording()

        if self.simulate:
            self.log(
                "Simulating eyelink, skipping data file closing",
                severity="WARN",
            )
            return

        self.log("Closing data file")
        self.tracker.setOfflineMode()
        self.tracker.closeDataFile()

        if self.session_bag_dir is None:
            self.log(
                "No session bag directory provided, skipping data file transfer",
                severity="WARN",
            )
            return

        received_dir = os.path.join(self.session_bag_dir, "eyelink_received")
        os.makedirs(received_dir, exist_ok=True)
        edf_path = os.path.join(received_dir, "last.edf")
        try:
            self.tracker.receiveDataFile("last.edf", edf_path)
        except Exception as e:
            raise DataFileReceiveError("Error receiving EDF file") from e

        self.log(f"Received EDF data file and saved to {edf_path}")

        try:
            csv_file_name = edf_to_csv(edf_path)
        except Exception as e:
            raise DataFileConversionError("Error converting EDF to CSV") from e

        self.log(f"Converted EDF to CSV: {csv_file_name}")

    def eye_available(self) -> EyeAvailable:
        """Get the eye available from the tracker.

        This function will return one of the following values:
        - LEFT_EYE: The left eye is available.
        - RIGHT_EYE: The right eye is available.
        - BINOCULAR: Both eyes are available.
        - NO_EYE: No eye is available.
        """
        if self.simulate:
            raise RuntimeError(
                "Simulating eyelink, cannot get eye availability"
            )

        return EyeAvailable(self.tracker.eyeAvailable())

    ###########################################################################
    # Sample retrieval
    ###########################################################################

    def sample_to_msg(self, sample: Sample, timestamp: Time) -> EyelinkMsg:
        """Get the valid eye data from the sample.

        This function will return the timestamp and the eye data for the left
        and right eyes, if available.

        Args:
            sample: The sample to get the eye data from.

        Returns:
            The eye data message, if available and valid. If only one eye is
            available/valid, its data will be set to MISSING_DATA. If no eye data is
            available/valid, the function will return None.
        """
        # self.log("Getting valid eye data", severity="DEBUG")

        left_sample: SampleData | None = sample.getLeftEye()
        right_sample: SampleData | None = sample.getRightEye()

        if left_sample is None:
            raise RuntimeError("Left eye sample is None")
        if right_sample is None:
            raise RuntimeError("Right eye sample is None")

        msg = EyelinkMsg()
        msg.header.stamp = timestamp.to_msg()
        msg.eyelink_time_ms = int(sample.getTime())
        msg.input = sample.getInput()

        msg.left_x, msg.left_y = left_sample.getRawPupil()
        msg.left_pupil = left_sample.getPupilSize()

        msg.right_x, msg.right_y = right_sample.getRawPupil()
        msg.right_pupil = right_sample.getPupilSize()

        return msg

    def generate_simulated_msg(
        self, min_pos: float, max_pos: float
    ) -> EyelinkMsg:
        msg = EyelinkMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.eyelink_time_ms = int(self.ros_time() / 1e3)
        data = np.random.randint(int(min_pos), int(max_pos), size=4)
        p = 1e-3
        prob = [p, 1 - p]
        msg.left_x = np.random.choice([MISSING_DATA, data[0]], p=prob)
        msg.left_y = np.random.choice([MISSING_DATA, data[1]], p=prob)
        msg.left_pupil = np.random.choice([MISSING_DATA, 5000], p=prob)
        msg.right_x = np.random.choice([MISSING_DATA, data[2]], p=prob)
        msg.right_y = np.random.choice([MISSING_DATA, data[3]], p=prob)
        msg.right_pupil = np.random.choice([MISSING_DATA, 5000], p=prob)
        msg.input = int(np.random.choice([255, 247]))
        return msg

    def sample_retrieval_loop(self):
        """Get samples from the tracker.

        This function will loop indefinitely, getting samples from the
        tracker and adding them to the sample queue. This function is
        thread-safe and should be run in a separate thread (or you'll not
        be able to do anything else).

        Args:
            stop_event: A threading event that can be used to stop the thread
            from another thread.
        """
        self.log("Starting sample retrieval loop")
        wait_for_data_timeout_ms = int(
            self.get_parameter_wrapper("wait_for_data_timeout") * 1e3
        )
        period = 1 / self.get_parameter_wrapper("sample_rate")
        config = self.preprocess_config["clean_eyelink"]
        min_pos = config["min_eye_pos"]
        max_pos = config["max_eye_pos"]
        try:
            # Wait for the tracker to be connected
            while (
                not self.simulate
                and not self.stop_sample_retrieval_event.is_set()
                and not self.tracker.isConnected()
            ):
                self.ros_sleep(period)

            # Receive data from the tracker, add it to the message queue, and
            # record it to the bag
            while not self.stop_sample_retrieval_event.is_set():
                start_time = self.ros_time()
                if self.simulate:
                    msg = self.generate_simulated_msg(min_pos, max_pos)
                    timestamp = self.get_clock().now()
                else:
                    try:
                        self.tracker.waitForData(
                            wait_for_data_timeout_ms, 1, 0
                        )
                    except RuntimeError as e:
                        self.log(
                            f"No data from tracker with error: {e}",
                            severity="WARN",
                        )
                        continue

                    sample: Sample | None = self.tracker.getNewestSample()
                    if sample is None or not isinstance(sample, Sample):  # type: ignore
                        msg = None
                    else:
                        timestamp = self.get_clock().now()
                        self.tracker.resetData()
                        msg = self.sample_to_msg(sample, timestamp)

                if msg is not None:
                    self.message_queue.append(msg)
                    if hasattr(self, "bag_writer"):
                        self.bag_writer.write(
                            "/eyelink/sample",
                            serialize_message(msg),  # type: ignore
                            timestamp.nanoseconds,  # type: ignore
                        )

                # Sleep for a short period to avoid busy-waiting (necessary
                # to force a context switch to other threads)
                sleep_time = period - (self.ros_time() - start_time)
                if self.simulate:
                    self.ros_sleep(0.9 * sleep_time)
                else:
                    self.ros_sleep(sleep_time)
        except ROSSleepError as e:
            if rclpy.ok():  # type: ignore
                raise RuntimeError("ROS2 is still running") from e

    ###########################################################################
    # Smooth pursuit
    ###########################################################################

    def _df_from_messages(self, msgs: list[EyelinkMsg]) -> pd.DataFrame:
        """Convert a list of Eyelink messages to a pandas dataframe."""
        return pd.DataFrame(
            (
                (
                    seconds_from_ros_time(msg.header.stamp),
                    msg.left_x,
                    msg.left_y,
                    msg.right_x,
                    msg.right_y,
                )
                for msg in msgs
            ),
            columns=["time", *EYELINK_DATA_COLS],  # type: ignore
        )

    def get_smooth_pursuit(self) -> bool:
        """Check if the monkey is smoothly pursuing.

        This function will check if the monkey is smoothly pursuing by
        checking if the speed of the left and right eyes is within the range
        provided by the min_speed and max_speed parameters.
        This is motivated by the fact that the eye can only move smoothly if it
        is following a smoothly moving object. Thus, if the speed of the eyes
        is below the minimum speed, we assume that the monkey is fixating on a
        static object. If the speed of the eyes is above the maximum speed, we
        assume that the monkey is saccading between objects. In either case, we
        assume that the monkey is not smoothly pursuing the desired target,
        which should be moving smoothly.

        Returns:
            True if the monkey is smoothly pursuing, False otherwise.
        """
        window = self.get_parameter_wrapper("smooth_pursuit.window")
        max_speed = self.get_parameter_wrapper("smooth_pursuit.max_speed")
        min_speed = self.get_parameter_wrapper("smooth_pursuit.min_speed")
        min_samples = self.get_parameter_wrapper("smooth_pursuit.min_samples")
        freq = self.get_parameter_wrapper("sample_rate")

        msgs = self.message_queue.to_list()
        start_time = self.ros_time()
        if len(msgs) < min_samples:
            self.log(
                f"Not enough samples in queue (min: {min_samples}, got: {len(msgs)})",
                severity="DEBUG",
            )
            return False

        # Convert messages to dataframe and filter out old samples
        df = self._df_from_messages(msgs)
        df = cast(pd.DataFrame, df[df["time"] > (start_time - window)])
        if df.shape[0] < min_samples:
            self.log(
                f"Not enough recent samples (min: {min_samples}, got: {df.shape[0]})",
                severity="DEBUG",
            )
            return False

        # Remove rows with missing data from the arrays
        config = self.preprocess_config["clean_eyelink"]
        df = clean_eyelink_data(df, **config)

        # Check if there are enough samples for meaningful smooth pursuit
        # extraction
        if df.shape[0] < min_samples:
            self.log(
                f"Not enough valid samples (min: {min_samples}, got: {df.shape[0]})",
                severity="DEBUG",
            )
            return False

        df = reindex_and_interpolate_eyelink_data(
            df,
            freq=freq,
            **self.preprocess_config["reindex_and_interpolate_eyelink"],
        )

        self.preprocess_config["smooth_eyelink"]["window"] = min(
            self.preprocess_config["smooth_eyelink"]["window"],
            df["time"].max() - df["time"].min(),
        )
        df = smooth_eyelink_data(
            df, freq=freq, **self.preprocess_config["smooth_eyelink"]
        )

        # if df.isna().any(axis=None):  # type: ignore
        #     self.log(
        #         "NaN values in dataframe, skipping smooth pursuit check",
        #         severity="DEBUG",
        #     )
        #     return False
        num_na = df.isna().any(axis=1).sum()  # type: ignore
        if num_na > 0:
            self.log(
                f"{num_na} NaN values in dataframe after smoothing",
                severity="DEBUG",
            )
        df = df.dropna()

        # Ensure that smooth pursuit is occuring by checking if the speeds of
        # the left and right eyes are below a threshold
        df = calculate_eyelink_speed(df)
        min_speed_calculated = df[["left_speed", "right_speed"]].min(axis=None)
        max_speed_calculated = df[["left_speed", "right_speed"]].max(axis=None)

        too_slow = min_speed_calculated < min_speed
        too_fast = max_speed_calculated > max_speed
        is_smoothly_pursuing = not (too_slow or too_fast)

        if is_smoothly_pursuing:
            self.log("Monkey is smoothly pursuing!", severity="DEBUG")
        else:
            if too_slow:
                self.log(
                    f"Monkey is too slow: {min_speed_calculated} < {min_speed}",
                    severity="DEBUG",
                )

            if too_fast:
                self.log(
                    f"Monkey is too fast: {max_speed_calculated} > {max_speed}",
                    severity="DEBUG",
                )

        # Log the smooth pursuit status and statistics about the eye speed data
        # if self.get_logger().get_effective_level() <= logging.DEBUG:
        self.log(
            f"Time taken: {self.ros_time() - start_time}", severity="DEBUG"
        )

        return is_smoothly_pursuing

    ###########################################################################
    # ROS callbacks
    ###########################################################################

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
        except Exception as e:
            response.success = False
            response.message = (
                f"Error closing data file: {type(e).__name__}: {e}"
            )
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

    def smooth_pursuit_goal_callback(self, _: Any) -> GoalResponse:
        with self.goal_callback_lock:
            if self.goal_ongoing:
                self.log(
                    "Cannot accept new goal, previous goal not finished",
                    severity="WARN",
                )
                return GoalResponse.REJECT
            else:
                self.goal_ongoing = True
                return GoalResponse.ACCEPT

    def smooth_pursuit_cancel_callback(self, _: Any) -> CancelResponse:
        with self.goal_callback_lock:
            if self.goal_ongoing:
                return CancelResponse.ACCEPT
            else:
                self.log(
                    "Cannot cancel goal, no goal in progress",
                    severity="WARN",
                )
                return CancelResponse.REJECT

    # Ideas: Continuously publish smooth pursuit state?
    #   I (don't) like this idea
    def smooth_pursuit_callback(
        self, goal_handle: ServerGoalHandle
    ) -> EyelinkSmoothPursuit.Result:
        """Flic response time action callback."""
        try:
            self.log("Starting smooth pursuit")
            window = self.get_parameter_wrapper("smooth_pursuit.window")
            last_smooth_pursuit = False

            while not goal_handle.is_cancel_requested:
                start_time = self.ros_time()
                smooth_pursuit = self.get_smooth_pursuit()

                goal_handle.publish_feedback(
                    EyelinkSmoothPursuit.Feedback(
                        is_smoothly_pursuing=smooth_pursuit
                    )
                )

                if smooth_pursuit != last_smooth_pursuit:
                    self.log(
                        f"Smooth pursuit {'started' if smooth_pursuit else 'ended'}",
                        severity="INFO",
                    )
                    last_smooth_pursuit = smooth_pursuit

                self.ros_sleep(window - (self.ros_time() - start_time))

            goal_handle.canceled()
            return EyelinkSmoothPursuit.Result()
        except ROSSleepError:
            self.log("ROS2 clock did not sleep correctly", severity="WARN")
            return EyelinkSmoothPursuit.Result()
        except Exception as e:
            self.log(
                f"Error in smooth pursuit callback: {e}", severity="ERROR"
            )
            raise
        finally:
            with self.goal_callback_lock:
                self.goal_ongoing = False

    def gaze_estimation_callback(self):
        """Callback to publish gaze estimation markers."""
        msgs = self.message_queue.to_list()

        x = None
        for msg in msgs:
            if (
                msg.left_x != MISSING_DATA
                and msg.left_y != MISSING_DATA
                and msg.right_x != MISSING_DATA
                and msg.right_y != MISSING_DATA
            ):
                x = torch.tensor(
                    [msg.left_x, msg.left_y, msg.right_x, msg.right_y],
                    dtype=torch.float32,
                ).unsqueeze(0)
                with torch.no_grad():
                    y = (
                        self.gaze_estimation_model(x)
                        .detach()
                        .squeeze()
                        .numpy()
                        .tolist()
                    )
                self.log(f"Gaze estimation: {y}", severity="DEBUG")
                break
        else:
            y = [0.0, 0.1, 0.0]
            self.log(
                f"No valid messages in queue, publishing default ({y})",
                severity="DEBUG",
            )

        markers = Markers()
        markers.header.stamp = self.get_clock().now().to_msg()
        markers.header.frame_id = "optitrack"

        markers.markers.append(  # type: ignore
            Marker(translation=Point(x=y[0], y=y[1], z=y[2]))
        )
        self.gaze_estimation_publisher.publish(markers)

    ###########################################################################
    # Node lifecycle
    ###########################################################################

    def destroy_node(self):
        """Destroy the node.

        This function will stop the sample retrieval loop, stop the recording,
        close the data file, and close the tracker.
        """

        try:
            self.log("Closing data file")
            self.close_data_file()
        except Exception as e:
            self.log(
                f"Error closing data file: {type(e).__name__}: {e}",
                severity="ERROR",
            )

        if hasattr(self, "bag_writer"):
            try:
                self.log("Closing bag writer")
                self.bag_writer.close()
            except Exception as e:
                self.log(
                    f"Error closing bag writer: {type(e).__name__}: {e}",
                    severity="ERROR",
                )

        # self.log("Shutting down thread pool")
        # try:
        #     self.tpe.shutdown()
        # except Exception as e:
        #     self.log(f"Error shutting down thread pool: {e}", severity="ERROR")

        if not self.simulate:
            self.log("Closing tracker")
            self.tracker.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Parse non-ROS arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug", action="store_true", default=False)

    non_ros_args = rclpy.utilities.remove_ros_args(args)  # type: ignore
    args, _ = parser.parse_known_args(non_ros_args)

    if args.debug:
        print("Debug mode enabled")
        debugpy.listen(1303)
        print("Waiting for debugger to attach")
        debugpy.wait_for_client()
        print("Debugger attached")

    try:
        executor: MultiThreadedExecutor | SingleThreadedExecutor = (
            MultiThreadedExecutor(num_threads=8)
        )
        eyelink = Eyelink()
        executor.add_node(eyelink)

        try:
            eyelink.open_data_file()
            eyelink.start_recording()
            print("Spinning")
            executor.spin()
        finally:
            print("Shutting down eyelink")
            eyelink.destroy_node()
            print("Shutting down executor")
            executor.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down rclpy")
        rclpy.try_shutdown()  # type: ignore


# TODO: Something is fucking wrong, help me
