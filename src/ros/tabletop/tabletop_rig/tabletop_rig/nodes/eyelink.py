"""ROS2 node for Eyelink eye tracker integration.

This module provides a ROS2 node that interfaces with SR Research Eyelink
eye trackers for gaze tracking in behavioral experiments. It handles:

- Communication with the Eyelink host PC over network
- Real-time sample streaming and recording
- Smooth pursuit detection for behavioral monitoring
- Optional gaze estimation using neural network models
- EDF file transfer and conversion to CSV

The node can operate in two modes:
- Real mode: Connects to actual Eyelink hardware via pylink
- Simulation mode: Generates synthetic gaze data for testing

Actions provided:
    ~/smooth_pursuit: Monitor and report smooth pursuit eye movements.

Topics published:
    ~/sample: Individual Eyelink sample messages (if not batched).
    ~/sample_array: Batched array of Eyelink samples (if batched mode).
    /predicted_markers: Gaze estimation predictions (if model enabled).

Parameters:
    tracker_address: IP address of the Eyelink host PC.
    do_tracker_setup: Whether to run calibration on startup.
    sample_rate: Sampling rate in Hz (default: 1000).
    session_bag_dir: Directory for saving recorded data.
    smooth_pursuit.window: Time window for pursuit detection (seconds).
    smooth_pursuit.min_samples: Minimum samples for valid pursuit detection.
    gaze_estimation.enable: Enable neural network gaze estimation.
    gaze_estimation.config: Path to gaze model configuration YAML.
    simulate: Run in simulation mode without hardware (default: false).

Example:
    ros2 run tabletop_rig eyelink --ros-args -p simulate:=true
"""

import argparse
import os
import threading
from collections import deque
from copy import copy
from enum import Enum
from typing import Any

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
from rclpy.event_handler import (
    PublisherEventCallbacks,
    QoSPublisherMatchedInfo,
)
from rclpy.exceptions import InvalidHandle, ParameterNotDeclaredException
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from tabletop_interfaces.action import EyelinkSmoothPursuit
from tabletop_interfaces.msg import Eyelink as EyelinkMsg
from tabletop_interfaces.msg import EyelinkArray as EyelinkArrayMsg

from tabletop_py.gaze.edf import edf_to_csv
from tabletop_py.gaze.preprocess import (
    EYELINK_DATA_COLS,
    calculate_eyelink_speed,
    clean_eyelink_data,
    reindex_and_interpolate,
    smooth_eyelink_data,
)
from tabletop_py.gaze.utils import (
    configure_torch_dtype,
    init_model,
    load_model_weights,
)
from tabletop_py.utils.common import dict_update_recursive
from tabletop_rig.executors import ErrorHandlingMultiThreadedExecutor
from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.ros import seconds_from_ros_time

try:
    PYLINK_AVAILABLE = True
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
    """Raised when EDF file transfer from Eyelink PC fails."""


class DataFileConversionError(Exception):
    """Raised when EDF to CSV conversion fails."""


class EyeAvailable(Enum):
    """Eye availability status from the Eyelink tracker.

    Attributes:
        NO_EYE: No eye data available (error state).
        LEFT_EYE: Only left eye is being tracked.
        RIGHT_EYE: Only right eye is being tracked.
        BINOCULAR: Both eyes are being tracked.
    """

    NO_EYE = -1
    LEFT_EYE = 0
    RIGHT_EYE = 1
    BINOCULAR = 2


class EyelinkMessageQueue:
    """Thread-safe bounded queue for Eyelink sample messages.

    Provides a circular buffer for storing recent Eyelink samples,
    used by the smooth pursuit detection algorithm. Thread-safe for
    concurrent producer/consumer access.

    Attributes:
        queue: Bounded deque storing EyelinkMsg messages.
        lock: Threading lock for synchronization.
    """

    def __init__(self, maxlen: int):
        """Initialize the message queue.

        Args:
            maxlen: Maximum number of messages to store.
        """
        self.queue = deque[EyelinkMsg](maxlen=maxlen)
        self.lock = threading.Lock()

    def append(self, msg: EyelinkMsg) -> None:
        """Add a message to the queue (thread-safe).

        Args:
            msg: Eyelink sample message to add.
        """
        with self.lock:
            self.queue.append(msg)

    def to_list(self) -> list[EyelinkMsg]:
        """Get all messages as a list (thread-safe).

        Returns:
            List of all messages currently in the queue.
        """
        with self.lock:
            return list(self.queue)

    def clear(self) -> None:
        """Remove all messages from the queue (thread-safe)."""
        with self.lock:
            self.queue.clear()


class Eyelink(BaseNode):
    """ROS2 node for Eyelink eye tracker integration.

    Manages communication with the Eyelink host PC, streams gaze samples,
    detects smooth pursuit eye movements, and optionally runs neural
    network-based gaze estimation.

    The node supports both real hardware operation and simulation mode
    for testing without an Eyelink system.

    Attributes:
        simulate: Whether running in simulation mode.
        tracker: Connection to the Eyelink host PC (real mode only).
        message_queue: Recent samples for smooth pursuit detection.
        recording: Whether currently recording samples.
    """

    default_params = BaseNode.default_params | {
        "tracker_address": "192.168.13.30",
        "do_tracker_setup": True,
        "simulate": False,
        "simulate_radius": 1000,
        "simulate_rotations_per_second": 1.0,
        "simulate_missing_prob": 1e-3,
        "simulate_saccate_prob": 1e-4,
        "wait_for_data_timeout": 0.1,  # seconds
        "sample_rate": 1000,  # Hz
        "publish_batched": True,
        "link_sample_data": "LEFT,RIGHT,RAW,AREA,INPUT,STATUS",
        "file_sample_data": "LEFT,RIGHT,RAW,AREA,INPUT,STATUS",
        "file_event_filter": "null",
        "link_event_filter": "null",
        "file_event_data": "null",
        "link_event_data": "null",
        "edf2asc_extra_args": ["-s", "-input", "-nflags", "-y"],
        "session_bag_dir": os.path.join(os.environ["ROS_BAG_DIR"], "latest"),
        "smooth_pursuit.window": 0.1,  # seconds
        "smooth_pursuit.min_samples": 80,
        # "preprocess_overrides.clean.max_zscore": "null",
        # "preprocess_overrides.reindex_and_interpolate.tolerance": "null",  # TODO: fix
        # "preprocess_overrides.reindex_and_interpolate.tolerance": 0.003,  # TODO: fix
        # "preprocess_overrides.smooth.window": 0.05,  # seconds
        "gaze_estimation.enable": True,
        "gaze_estimation.frame_id": "optitrack",
        "gaze_estimation.device": "cpu",
        "gaze_estimation.compile": False,
        "gaze_estimation.config": "$TABLETOP_DIR/config/gaze_estimation.yaml",
        "gaze_estimation.freq": 100,  # Hz
        "gaze_estimation.window": 0.05,  # seconds
    }

    ###########################################################################
    # Initialization
    ###########################################################################

    def __init__(self):
        """Initialize the Eyelink node.

        Sets up the tracker connection (or simulation), sample queue,
        bag writer, gaze estimation model, and ROS interfaces.

        Raises:
            RuntimeError: If pylink not available and not in simulation mode.
        """
        super().__init__("eyelink")

        self.init_tracker()

        self.init_state()

        self.init_gaze_estimation()

        self.init_ros()

    def init_tracker(self) -> None:
        self.simulate = self.param("simulate")

        if not self.simulate:
            if not PYLINK_AVAILABLE:
                raise RuntimeError(
                    "pylink module not available, make sure the Eyelink and "
                    "Pylink libraries are installed or set simulate to true"
                )
            self.log("Pylink available, connecting to Eyelink machine")
            self.tracker = EyeLinkTracker(  # type: ignore
                self.param("tracker_address")
            )
            self.tracker.stopRecording()
            self._close_data_file(save=False)
        else:
            self.log("Simulating eyelink data...")

    def init_state(self) -> None:
        """Initialize sample retrieval infrastructure.
        Sets up the message queue for recent samples, threading primitives
        for the sample retrieval loop, and the recording state flag.
        """
        sample_rate = self.param("sample_rate")
        self.smooth_pursuit_window = self.param("smooth_pursuit.window")

        self.message_queue = EyelinkMessageQueue(
            maxlen=int(sample_rate * self.smooth_pursuit_window)
        )
        # self.tpe = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.stop_sample_retrieval_event = threading.Event()
        self.stop_sample_retrieval_event.set()

        self._goal_lock = threading.Lock()
        self._goal_ongoing = False

        self._retrieval_lock = threading.Lock()
        self._retrieving = False

        self._subscribers_lock = threading.Lock()
        self._has_subscribers = False

    def init_gaze_estimation(self) -> None:
        """Initialize the neural network gaze estimation model.

        Loads the model configuration and weights for real-time gaze
        prediction from raw eye position data.

        Raises:
            ValueError: If sample rate doesn't match model configuration.
        """
        path = os.path.expandvars(
            os.path.expanduser(self.param("gaze_estimation.config"))
        )
        with open(path, "r") as f:
            self.gaze_estimation_config = yaml.safe_load(f)

        sample_rate = self.param("sample_rate")
        eyelink_freq = self.gaze_estimation_config["eyelink_freq"]
        if sample_rate != eyelink_freq:
            raise ValueError(
                f"Sample rate ({sample_rate}) and gaze estimation eyelink frequency ({eyelink_freq}) must be the same"
            )

        self.preprocess_config = self.gaze_estimation_config["preprocess"][
            "eyelink"
        ]
        try:
            overrides = self.param("preprocess_overrides")
        except ParameterNotDeclaredException:
            pass
        else:
            self.preprocess_config = dict_update_recursive(
                self.preprocess_config, overrides
            )

        if self.param("gaze_estimation.enable"):
            weights_path = os.path.expandvars(
                os.path.expanduser(self.gaze_estimation_config["weights_path"])
            )
            if not os.path.exists(weights_path):
                self.log(
                    f"weights_path ({weights_path}) does not exist, "
                    f"skipping live gaze estimation initialization",
                    severity="WARN",
                )
                return

            device = self.param("gaze_estimation.device")
            if device is None:
                device = torch.device(
                    "cuda" if torch.cuda.is_available() else "cpu"
                )
            else:
                device = torch.device(device)

            self.log(f"Using device '{device}' for live gaze estimation")

            configure_torch_dtype()

            self.gaze_estimation_model = init_model(
                **self.gaze_estimation_config["model"]
            ).to(device)

            load_model_weights(
                self.gaze_estimation_model, weights_path, device
            )
            # self.gaze_estimation_model.compile(
            #     **self.gaze_estimation_config["compile"]
            # )
            self.gaze_estimation_model.eval()
            self.last_gaze_estimation_time = self.ros_time()

    def init_ros(self) -> None:
        """Initialize ROS2 interfaces.

        Creates services for recording control, the smooth pursuit action
        server, and optionally the gaze estimation publisher and timer.
        """
        event_callbacks = PublisherEventCallbacks(
            matched=self.sample_publisher_matched_callback
        )
        if self.param("publish_batched"):
            self.sample_publisher = self.create_publisher(
                EyelinkArrayMsg,
                "~/sample_array",
                10,
                callback_group=MutuallyExclusiveCallbackGroup(),
                event_callbacks=event_callbacks,
            )
        else:
            # qos = copy(QoSPresetProfiles.SENSOR_DATA.value)
            # qos.durability = QoSDurabilityPolicy.VOLATILE
            # qos.depth = 500
            self.sample_publisher = self.create_publisher(
                EyelinkMsg,
                "~/sample",
                500,
                callback_group=MutuallyExclusiveCallbackGroup(),
                event_callbacks=event_callbacks,
            )

        self.smooth_pursuit_server = ActionServer(
            self,
            EyelinkSmoothPursuit,
            "~/smooth_pursuit",
            self.smooth_pursuit_callback,
            cancel_callback=self.smooth_pursuit_cancel_callback,
            goal_callback=self.smooth_pursuit_goal_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),  # TODO: Fix callback groups
        )

        if hasattr(self, "gaze_estimation_model"):
            self.gaze_estimation_publisher = self.create_publisher(
                Markers,
                "/predicted_markers",
                qos_profile=1000,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
            self.gaze_estimation_timer = self.create_timer(
                1 / self.param("gaze_estimation.freq"),
                self.gaze_estimation_callback,
                callback_group=MutuallyExclusiveCallbackGroup(),
                autostart=False,
            )

    @property
    def session_bag_dir(self) -> str | None:
        return self.param("session_bag_dir")

    @property
    def goal_ongoing(self) -> bool:
        with self._goal_lock:
            return self._goal_ongoing

    @property
    def has_subscribers(self) -> bool:
        with self._subscribers_lock:
            return self._has_subscribers

    ###########################################################################
    # Tracker utilities
    ###########################################################################

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

    def eyelink_pc_setup(self) -> None:
        """Run interactive tracker setup/calibration on the Eyelink PC.

        Initiates the calibration procedure on the Eyelink host PC.
        This is a blocking call that waits until the operator presses
        ESC on the Eyelink PC to complete setup.

        Note:
            This blocks the Python process and won't respond to Ctrl+C.
            Must press ESC on the Eyelink PC to continue.

        Raises:
            RuntimeError: If called in simulation mode.
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

    def _open_bag_writer(self, session_bag_dir: str | None) -> None:
        """Initialize the rosbag writer for recording samples.

        Creates a bag writer in the session directory if configured.
        Samples are recorded in MCAP format for later analysis.

        Raises:
            ValueError: If session_bag_dir is set but not a valid directory.
        """
        self.log("Opening bag writer")
        if session_bag_dir is None:
            self.log(
                "No session bag directory provided, skipping bag writer",
                severity="WARN",
            )
            return

        if not os.path.isdir(session_bag_dir):
            raise ValueError(
                f"Session bag directory {session_bag_dir} is not a directory"
            )

        bag_dir = os.path.join(session_bag_dir, "eyelink")
        self.bag_writer = rosbag2_py.SequentialWriter()
        try:
            storage_options = rosbag2_py.StorageOptions(
                uri=bag_dir, storage_id="mcap"
            )
            converter_options = rosbag2_py.ConverterOptions("", "")
            self.bag_writer.open(storage_options, converter_options)
            self.bag_writer.create_topic(
                rosbag2_py.TopicMetadata(
                    id=0,
                    name="~/sample",
                    type="tabletop_interfaces/msg/Eyelink",
                    serialization_format="cdr",
                )
            )
        except:
            self._close_bag_writer()
            raise

    def _close_bag_writer(self) -> None:
        """Close the rosbag writer.

        Flushes and closes the rosbag2 writer if one exists.
        """
        self.log("Closing bag writer")
        if hasattr(self, "bag_writer"):
            self.bag_writer.close()
            del self.bag_writer

    def _open_data_file(self) -> None:
        """Open an EDF data file on the Eyelink host PC.

        Creates 'last.edf' on the Eyelink PC to store sample data.
        Configures data collection parameters from node settings.
        """
        self.log("Opening data file")
        edf_file_name = "last.edf"
        self.tracker.openDataFile(edf_file_name)
        try:
            preamble_text = "RECORDED BY EyeLink ROS Node"
            self.tracker.sendCommand(
                f"add_file_preamble_text '{preamble_text}'"
            )

            self.tracker.setPupilSizeDiameter("YES")

            for key in [
                "file_sample_data",
                "link_sample_data",
                "file_event_filter",
                "link_event_filter",
                "file_event_data",
                "link_event_data",
            ]:
                value = self.param(key)
                if value is not None:
                    self.tracker.sendCommand(f"{key} = {value}")
        except:
            self._close_data_file(save=False)
            raise

    def _close_data_file(self, *, save: bool) -> None:
        """Close the EDF file and transfer to local machine.

        Stops recording, closes the file on the Eyelink PC, transfers
        it to the local session directory, and converts to CSV format.

        Raises:
            DataFileReceiveError: If file transfer fails.
            DataFileConversionError: If EDF to CSV conversion fails.
        """
        self.log("Closing data file")
        self.tracker.setOfflineMode()
        self.tracker.closeDataFile()

        if not save:
            return

        if self.session_bag_dir is None or not os.path.exists(
            self.session_bag_dir
        ):
            self.log(
                f"session_bag_dir ({self.session_bag_dir}) not provided or "
                f"does not exist, skipping data file transfer",
                severity="WARN",
            )
            return

        received_dir = os.path.join(self.session_bag_dir, "eyelink_received")
        edf_path = os.path.join(received_dir, "last.edf")

        if os.path.exists(edf_path):
            self.log(
                f"{edf_path} already exists, skipping data file transfer",
                severity="WARN",
            )
            return

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

    def _start_sample_retrieval_loop(self) -> None:
        """Start the background sample retrieval task.

        Launches the sample retrieval loop as an executor task
        that continuously reads samples from the tracker.
        """
        self.log("Starting sample retrieval loop")
        assert self.stop_sample_retrieval_event.is_set()
        assert not hasattr(self, "sample_retrieval_future"), (
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
        # self.sample_retrieval_future.add_done_callback(
        #     lambda _: self._wake_executor()
        # )

    def _stop_sample_retrieval_loop(self) -> None:
        """Stop the background sample retrieval task.

        Signals the retrieval loop to stop and waits for completion.
        """
        self.log("Stopping sample retrieval loop")
        self.stop_sample_retrieval_event.set()
        if hasattr(self, "sample_retrieval_future"):
            try:
                self.sample_retrieval_future.result()
            except Exception as e:
                self.log(
                    f"Error in sample retrieval loop during stop: {type(e).__name__}: {e}",
                    severity="ERROR",
                )
            finally:
                del self.sample_retrieval_future

    def start_retrieval(self) -> None:
        """Start recording eye-gaze samples from the Eyelink tracker.

        Opens the EDF data file, begins the sample retrieval loop,
        and starts hardware recording. Initializes gaze estimation
        timer if model is available.

        Raises:
            RuntimeError: If recording fails to start or eye availability
                is not binocular.
        """
        with self._retrieval_lock:
            self.log("Starting sample retrieval")

            if self._retrieving:
                self.log(
                    "Sample retrieval ongoing, skipping sample retrieval start",
                    severity="WARN",
                )
                return

            # Data file must be opened before starting recording
            if not self.simulate:
                self._open_data_file()
            try:
                # Start sample retrieval loop in separate thread
                self._start_sample_retrieval_loop()
                try:
                    if self.simulate:
                        self.log(
                            "Simulating eyelink, skipping recording start",
                            severity="WARN",
                        )
                    else:
                        self.log("Starting recording")
                        self.tracker.setOfflineMode()
                        if (
                            ret := self.tracker.startRecording(1, 0, 1, 0)
                        ) != 0:
                            raise RuntimeError(
                                f"Eyelink start recording failed with error code: {ret}"
                            )
                        # pylink.beginRealTimeMode(100)  # pyright: ignore[reportPossiblyUnboundVariable]
                        # if self.tracker.getSampleRate() != self.param(
                        #     "sample_rate"
                        # ):
                        #     self.log(
                        #         f"Tracker sample rate ({self.tracker.getSampleRate()}) "
                        #             f"does not equal expected sample rate ({self.param('sample_rate')})", sev
                        #     )
                        # self.tracker.sendMessage("SYNCTIME")
                        self.tracker.sendMessage("SYNCTIME")
                        eye_available = self.eye_available()
                        if eye_available != EyeAvailable.BINOCULAR:
                            raise RuntimeError(
                                f"Only binocular mode is supported, got {eye_available}"
                            )
                except Exception:
                    self._stop_sample_retrieval_loop()
                    raise
            except Exception:
                # self.tracker.stopRecording()  # TODO: maybe fix
                self.tracker.stopRecording()  # TODO: maybe fix
                self._close_data_file(save=False)
                raise

            # Start gaze_estimation_timer
            if hasattr(self, "gaze_estimation_timer"):
                self.gaze_estimation_timer.reset()

            self._retrieving = True

    def stop_retrieval(self, force: bool = False) -> None:
        """Stop recording eye-gaze samples and close the data file.

        Halts the sample retrieval loop and closes the EDF file on the
        Eyelink PC, transferring it to local storage if successful.
        Can be forced to stop even if goals or subscribers are active.

        Args:
            force: If True, stop retrieval even if goals or subscribers
                are active. If False, skip stopping if still needed.
        """
        with self._retrieval_lock:
            self.log("Stopping sample retrieval")
            if not self._retrieving:
                self.log(
                    "Not retrieving, skipping sample retrieval stop",
                    severity="WARN",
                )
                return

            if not force and (self.goal_ongoing or self.has_subscribers):
                self.log(
                    "Retrieval still needed, can't stop yet",
                    severity="WARN",
                )
                return

            # Stop gaze_estimation_timer
            if hasattr(self, "gaze_estimation_timer"):
                try:
                    self.gaze_estimation_timer.cancel()
                except InvalidHandle:
                    pass

            # Stop sample retrieval before stopping recording
            self._stop_sample_retrieval_loop()

            if self.simulate:
                self.log(
                    "Simulating eyelink, skipping recording stop",
                    severity="WARN",
                )
            else:
                self.tracker.stopRecording()
                try:
                    self._close_data_file(save=True)
                except (DataFileReceiveError, DataFileConversionError) as e:
                    self.log(
                        f"Error processing datafile: {type(e).__name__}: {e}",
                        severity="ERROR",
                    )

            self._retrieving = False

    ###########################################################################
    # Sample retrieval
    ###########################################################################

    def sample_to_msg(self, sample: Sample, timestamp: Time) -> EyelinkMsg:
        """Convert an Eyelink sample to a ROS message.

        Extracts raw pupil positions and sizes from both eyes
        and packages them into an EyelinkMsg.

        Args:
            sample: The pylink Sample object from the tracker.
            timestamp: ROS timestamp for the message header.

        Returns:
            Populated EyelinkMsg with eye position data.

        Raises:
            RuntimeError: If left or right eye sample is None.
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
        """Generate a synthetic Eyelink sample for simulation mode.

        Creates realistic circular eye movement patterns with occasional
        missing data and saccades to test the processing pipeline.

        Args:
            min_pos: Minimum eye position value.
            max_pos: Maximum eye position value.

        Returns:
            Simulated EyelinkMsg with synthetic eye data.
        """
        msg = EyelinkMsg()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        t = seconds_from_ros_time(now)
        msg.eyelink_time_ms = int(t * 1e3)

        center = np.mean([min_pos, max_pos])
        radius = self.param("simulate_radius")
        rps = self.param("simulate_rotations_per_second")
        t = np.full(4, t, dtype=np.float64)
        phi = np.array([0, np.pi / 2, 0, np.pi / 2])
        pos = np.sin(2 * np.pi * t * rps + phi) * radius + center
        pos = np.round(np.clip(pos, min_pos, max_pos))

        p_missing = self.param("simulate_missing_prob")
        p_saccate = self.param("simulate_saccate_prob")
        p_normal = 1 - (p_missing + p_saccate)

        mask = np.random.choice(
            [0, 1, 2], size=4, p=[p_normal, p_missing, p_saccate]
        )
        masked_pos = (
            (mask == 0) * pos
            + (mask == 1) * MISSING_DATA
            + (mask == 2) * min_pos
        )

        msg.left_x = masked_pos[0]
        msg.left_y = masked_pos[1]
        msg.left_pupil = np.random.choice(
            [5000, MISSING_DATA], p=[p_normal + p_saccate, p_missing]
        )
        msg.right_x = masked_pos[2]
        msg.right_y = masked_pos[3]
        msg.right_pupil = np.random.choice(
            [5000, MISSING_DATA], p=[p_normal + p_saccate, p_missing]
        )
        msg.input = int(np.random.choice([255, 247]))

        return msg

    def sample_retrieval_loop(self) -> None:
        """Main loop for retrieving samples from the tracker.

        Continuously polls the tracker for new samples, converts them
        to ROS messages, adds them to the message queue, and writes
        them to the rosbag. Runs until stop_sample_retrieval_event is set.

        In simulation mode, generates synthetic data instead.
        """
        self.log("Entered sample retrieval loop")
        wait_for_data_timeout_ms = int(
            self.param("wait_for_data_timeout") * 1e3
        )
        period: float = 1 / self.param("sample_rate")
        publish_batched: bool = self.param("publish_batched")

        array_msg = EyelinkArrayMsg()
        array_idx = 0
        array_len = len(array_msg.samples)

        assert isinstance(array_msg.samples, list)
        assert array_len > 0

        min_pos = self.preprocess_config["clean"]["min_eye_pos"]
        max_pos = self.preprocess_config["clean"]["max_eye_pos"]

        # Wait for the tracker to be connected
        # while (
        #     not self.simulate
        #     and not self.stop_sample_retrieval_event.is_set()
        #     and not self.tracker.isConnected()
        # ):
        #     self.ros_sleep(period)

        # TODO: Discard old samples before starting collection

        while not self.stop_sample_retrieval_event.is_set():
            # Receive data from the tracker and convert to ROS message if valid
            start_time = self.ros_time()
            if self.simulate:
                msg = self.generate_simulated_msg(min_pos, max_pos)
            else:
                try:
                    self.tracker.waitForData(wait_for_data_timeout_ms, 1, 0)
                except RuntimeError as e:
                    # TODO: Verify this fix. MAY need to explicitly get
                    # the logger to do throttled logging
                    # so that rclpy knows the "caller id" to throttle
                    # self.get_logger().warning(
                    #     f"No data from tracker with error: {e}",
                    #     throttle_duration_sec=1,
                    # )
                    self.log(
                        f"No data from tracker with error: {e}",
                        severity="WARN",
                        throttle_duration_sec=1,
                    )
                    continue
                timestamp = self.get_clock().now()
                sample: Sample | None = self.tracker.getNewestSample()
                if sample is None or not isinstance(sample, Sample):  # type: ignore
                    msg = None
                else:
                    self.tracker.resetData()
                    msg = self.sample_to_msg(sample, timestamp)

            # Add the message to the queue and record it to the bag
            if msg is not None:
                self.message_queue.append(msg)
                if self.has_subscribers:
                    if publish_batched:
                        array_msg.samples[array_idx] = msg  # type: ignore
                        array_idx += 1
                        if array_idx == array_len:
                            self.sample_publisher.publish(array_msg)
                            array_msg = EyelinkArrayMsg()
                            array_idx = 0
                    else:
                        self.sample_publisher.publish(msg)
                # if hasattr(self, "bag_writer"):
                #     self.bag_writer.write(
                #         "/eyelink/sample",
                #         serialize_message(msg),  # type: ignore
                #         timestamp.nanoseconds,  # type: ignore
                #     )

            # Sleep for a short period to avoid busy-waiting (necessary
            # to force a context switch to other threads)
            taken = self.ros_time() - start_time
            if taken < period:
                if self.simulate:
                    # time.sleep(period - taken)
                    self.ros_sleep(period - taken)
                else:
                    # time.sleep(0.95 * (period - taken))
                    self.ros_sleep(0.95 * (period - taken))
            else:
                self.log(
                    f"Sample retrieval took longer than expected period: {taken:.4f}s > {period:.4f}s",
                    severity="WARN",
                )
                self.ros_sleep(0)
        # except (ROSSleepError, NotInitializedException) as e:
        #     raise e
        #     if rclpy.ok():  # type: ignore
        #         raise RuntimeError("ROS2 is still running") from e

    ###########################################################################
    # Smooth pursuit
    ###########################################################################

    def _df_from_messages(self, msgs: list[EyelinkMsg]) -> pd.DataFrame:
        """Convert Eyelink messages to a DataFrame for analysis.

        Args:
            msgs: List of Eyelink sample messages.

        Returns:
            DataFrame with columns: time, left_x, left_y, right_x, right_y.
        """
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
        ).astype(float)

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
        window = self.param("smooth_pursuit.window")
        min_samples = self.param("smooth_pursuit.min_samples")
        freq = self.param("sample_rate")

        msgs = self.message_queue.to_list()
        start_time = self.ros_time()
        if len(msgs) < min_samples:
            self.log(
                f"Not enough samples in queue (min: {min_samples}, got: {len(msgs)})",
                severity="WARN",
            )
            return False

        # Convert messages to dataframe and filter out old samples
        df = self._df_from_messages(msgs)
        df = df[df["time"] > (start_time - window)]
        if df.shape[0] < min_samples:
            self.log(
                f"Not enough recent samples (min: {min_samples}, got: {df.shape[0]})",
                severity="WARN",
            )
            return False

        # Remove rows with missing data from the arrays
        df = clean_eyelink_data(df, **self.preprocess_config["clean"])

        # Check if there are enough samples for meaningful smooth pursuit
        # extraction
        if df.shape[0] < min_samples:
            self.log(
                f"Not enough valid samples (min: {min_samples}, got: {df.shape[0]})",
                severity="WARN",
            )
            return False

        steady_idx = np.arange(df["time"].min(), df["time"].max(), 1 / freq)

        df = reindex_and_interpolate(
            df,
            steady_idx,
            on="time",
            **self.preprocess_config["reindex_and_interpolate"],
        )
        num_na = df.isna().any(axis=1).sum()
        if num_na > 0:
            self.log(
                f"{num_na} NaN values in dataframe after reindexing and interpolating",
                severity="WARN",
            )
            return False

        smooth_config = copy(self.preprocess_config["smooth"])
        smooth_config["window"] = min(
            smooth_config["window"], df["time"].max() - df["time"].min()
        )
        df = smooth_eyelink_data(df, freq=freq, **smooth_config)

        num_na = df.isna().any(axis=1).sum()
        df = df.dropna()

        # Ensure that smooth pursuit is occuring by checking if the speeds of
        # the left and right eyes are below a threshold
        speed_df = calculate_eyelink_speed(df)
        min_speed = self.preprocess_config["filter_by_speed"]["min_speed"]
        max_speed = self.preprocess_config["filter_by_speed"]["max_speed"]
        min_speed_calculated = speed_df.min(axis=None)
        max_speed_calculated = speed_df.max(axis=None)

        too_slow = min_speed_calculated < min_speed
        too_fast = max_speed_calculated > max_speed
        is_smoothly_pursuing = not (too_slow or too_fast)

        if is_smoothly_pursuing:
            self.log("Monkey is smoothly pursuing!")
        else:
            if too_slow:
                self.log(
                    f"Monkey is too slow: {min_speed_calculated} < {min_speed}"
                )

            if too_fast:
                self.log(
                    f"Monkey is too fast: {max_speed_calculated} > {max_speed}"
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

    def sample_publisher_matched_callback(self, info: QoSPublisherMatchedInfo):
        """Handle sample publisher subscription changes.

        Automatically starts sample retrieval when subscribers appear
        and stops when the last subscriber disconnects.

        Args:
            info: QoS event containing subscriber count.
        """
        if info.current_count != 0:
            self.log(f"Sample publisher has {info.current_count} subscribers")
            with self._subscribers_lock:
                self._has_subscribers = True
            self.start_retrieval()
        else:
            self.log("Sample publisher has no subscribers")
            with self._subscribers_lock:
                self._has_subscribers = False
            self.stop_retrieval()

    def smooth_pursuit_goal_callback(self, _: Any) -> GoalResponse:
        """Handle incoming smooth pursuit action goals.

        Rejects if another goal is already in progress.

        Args:
            _: Unused goal request.

        Returns:
            GoalResponse.ACCEPT or GoalResponse.REJECT.
        """
        with self._goal_lock:
            if self._goal_ongoing:
                self.log(
                    "Cannot accept new goal, previous goal not finished",
                    severity="WARN",
                )
                return GoalResponse.REJECT
            else:
                self._goal_ongoing = True
                return GoalResponse.ACCEPT

    def smooth_pursuit_cancel_callback(self, _: Any) -> CancelResponse:
        """Handle smooth pursuit action cancellation requests.

        Args:
            _: Unused cancel request.

        Returns:
            CancelResponse.ACCEPT or CancelResponse.REJECT.
        """
        with self._goal_lock:
            if self._goal_ongoing:
                return CancelResponse.ACCEPT
            else:
                self.log(
                    "Cannot cancel goal, no goal in progress",
                    severity="WARN",
                )
                return CancelResponse.REJECT

    def smooth_pursuit_callback(
        self, goal_handle: ServerGoalHandle
    ) -> EyelinkSmoothPursuit.Result:
        """Execute smooth pursuit monitoring action.

        Periodically checks for smooth pursuit and publishes feedback
        until cancelled.

        Args:
            goal_handle: The action goal handle.

        Returns:
            Empty result (action typically cancelled by client).
        """
        try:
            self.log("Starting smooth pursuit")
            self.start_retrieval()
            window = self.param("smooth_pursuit.window")
            last_smooth_pursuit = False

            while (
                goal_handle.is_active
                and not goal_handle.is_cancel_requested
                and rclpy.ok()  # type: ignore
            ):
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

                taken = self.ros_time() - start_time
                if taken < window:
                    self.ros_sleep(window - taken)
                else:
                    self.log(
                        f"Smooth pursuit loop took longer than window: {taken:.4f}s > {window:.4f}s",
                        severity="WARN",
                    )
                    self.ros_sleep(0)
        except Exception:
            goal_handle.abort()
            raise
        else:
            if goal_handle.is_active and goal_handle.is_cancel_requested:
                self.log("Goal canceled")
                goal_handle.canceled()
            return EyelinkSmoothPursuit.Result()
        finally:
            with self._goal_lock:
                self._goal_ongoing = False
            self.stop_retrieval()

    def gaze_estimation_callback(self) -> None:
        """Publish gaze estimation predictions.

        Runs the neural network model on the latest valid sample
        and publishes the predicted 3D gaze point as a Marker.
        """
        msgs = self.message_queue.to_list()
        now = self.ros_time()
        window = self.param("gaze_estimation.window")
        frame_id = self.param("gaze_estimation.frame_id")

        y = None
        stamp = None
        for msg in reversed(msgs):
            if now - seconds_from_ros_time(msg.header.stamp) > window:
                self.log(
                    f"No valid messages within gaze estimation window ({window}) in queue",
                    severity="DEBUG",
                )
                break
            if (
                seconds_from_ros_time(msg.header.stamp)
                <= self.last_gaze_estimation_time + 1e-5
            ):
                self.log(
                    "No new valid messages in queue",
                    severity="DEBUG",
                )
                break
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
                stamp = msg.header.stamp
                self.last_gaze_estimation_time = seconds_from_ros_time(
                    msg.header.stamp
                )
                self.log(f"Gaze estimation: {y}", severity="DEBUG")
                break
        else:
            self.log("No valid messages in queue", severity="DEBUG")

        markers = Markers()
        markers.header.frame_id = frame_id

        if y is not None:
            assert stamp is not None
            markers.header.stamp = stamp
            markers.markers.append(  # type: ignore
                Marker(translation=Point(x=y[0], y=y[1], z=y[2]))
            )
        else:
            markers.header.stamp = self.get_clock().now().to_msg()

        self.last_gaze_estimation_time = seconds_from_ros_time(
            markers.header.stamp
        )

        self.gaze_estimation_publisher.publish(markers)

    ###########################################################################
    # Node lifecycle
    ###########################################################################

    def destroy_node(self) -> None:
        """Clean up resources and destroy the node.

        Stops recording, closes the bag writer, and disconnects
        from the Eyelink tracker before calling parent destroy.
        """

        try:
            self.stop_retrieval(force=True)
        except Exception as e:
            self.log(
                f"Error stopping recording: {type(e).__name__}: {e}",
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

        if hasattr(self, "smooth_pursuit_server"):
            self.smooth_pursuit_server.destroy()
        super().destroy_node()


def main(args=None):
    """Entry point for the eyelink node."""
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
        executor: (
            ErrorHandlingMultiThreadedExecutor | SingleThreadedExecutor
        ) = ErrorHandlingMultiThreadedExecutor()
        eyelink = Eyelink()
        executor.add_node(eyelink)

        try:
            # eyelink.start_recording(eyelink.param("session_bag_dir"))
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
