import concurrent.futures
import os
import threading
from collections import deque
from copy import copy
from datetime import datetime
from enum import Enum

import numpy as np
import rclpy
import rosbag2_py
from pylink import EyeLink as EyeLinkTracker
from pylink.constants import MISSING_DATA
from pylink.tracker import Sample, SampleData
from rclpy.action.server import (
    ActionServer,
    CancelResponse,
    GoalResponse,
    ServerGoalHandle,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.exceptions import NotInitializedException
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSPresetProfiles
from rclpy.serialization import serialize_message
from rclpy.time import Time
from std_srvs.srv import Trigger

from tabletop_interfaces.action import EyelinkSmoothPursuit
from tabletop_interfaces.msg import Eyelink as EyelinkMsg
from tabletop_server.nodes.base import BaseNode
from tabletop_utils.eyelink import edf_to_csv
from tabletop_utils.ros import ROSSleepError


class DataFileReceiveError(Exception):
    """Error while receiving the data file."""


class DataFileConversionError(Exception):
    """Error while converting the data file."""


class EyeAvailable(Enum):
    NO_EYE = -1
    LEFT_EYE = 0
    RIGHT_EYE = 1
    BINOCULAR = 2


class Eyelink(BaseNode):
    default_params = BaseNode.default_params | {
        "tracker_address": "192.168.13.30",
        "do_tracker_setup": True,
        "wait_for_data_timeout": 0.2,  # seconds
        "sample_rate": 1000,  # Hz
        "max_window": 0.5,  # seconds
        "link_sample_data": "LEFT,RIGHT,RAW,AREA,INPUT,STATUS",
        "file_sample_data": "LEFT,RIGHT,RAW,AREA,INPUT,STATUS",
        "file_event_filter": "null",
        "link_event_filter": "null",
        "file_event_data": "null",
        "link_event_data": "null",
        "session_bag_dir": "null",
        "edf2asc_extra_args": ["-s", "-input", "-nflags", "-y"],
        "log_samples": False,
        "log_smooth_pursuit": True,
        "log_smooth_pursuit_window": 0.5,  # seconds
        "log_smooth_pursuit_threshold": 4e4,  # pixels/s
        "log_smooth_pursuit_min_samples": 75,
        "log_period": 0.1,  # seconds
    }

    ###########################################################################
    # Initialization
    ###########################################################################

    def __init__(self):
        super().__init__("eyelink")

        self.tracker = EyeLinkTracker(
            self.get_parameter_wrapper("tracker_address")
        )
        # pylink.endRealTimeMode()

        self.init_ros()
        self.init_sample_retrieval()

        self.destroyed = False

    def init_ros(self):
        """Setup the ROS node.

        This function will setup the ROS node. It will create a timer to log
        the eyelink status and a series of services to control the tracker.
        """
        # self.log_timer = self.create_timer(
        #     self.get_parameter_wrapper("log_period"),
        #     self.log_callback,
        #     callback_group=MutuallyExclusiveCallbackGroup(),
        # )

        # Publishers
        qos = copy(QoSPresetProfiles.SENSOR_DATA.value)
        qos.durability = QoSDurabilityPolicy.VOLATILE
        qos.depth = 10
        # self.eyelink_sample_publisher = self.create_publisher(
        #     EyelinkMsg,
        #     "/eyelink/sample",
        #     qos_profile=qos,
        #     callback_group=MutuallyExclusiveCallbackGroup(),
        # )

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

    def init_sample_retrieval(self):
        """Setup the sample retrieval.

        This function will setup the sample queue and thread pool, as well as
        the stop event and sample retrieval loop future.
        """
        sample_rate = self.get_parameter_wrapper("sample_rate")
        self.max_window = self.get_parameter_wrapper("max_window")
        self.message_queue: deque[EyelinkMsg] = deque(
            maxlen=int(sample_rate * self.max_window)
        )
        self.message_queue_lock = threading.Lock()
        # self.tpe = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.stop_sample_retrieval_event = threading.Event()
        self.stop_sample_retrieval_event.set()
        self.sample_retrieval_future = concurrent.futures.Future()
        self.sample_retrieval_future.set_result(None)
        self.recording = False

        # Create session bag directory
        self.session_bag_dir = self.get_parameter_wrapper("session_bag_dir")
        if self.session_bag_dir is None:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            dirname = f"eyelink_{timestamp}"
            self.session_bag_dir = os.path.join(
                os.environ["ROS_BAG_DIR"], dirname
            )
            os.makedirs(self.session_bag_dir)
        elif not os.path.isdir(self.session_bag_dir):
            raise ValueError(
                f"Session bag directory {self.session_bag_dir} is not a directory"
            )

        # Setup bag writer
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
        assert self.sample_retrieval_future.done(), "Sample retrieval already running, may be in the process of stopping"

        with self.message_queue_lock:
            self.message_queue.clear()

        self.stop_sample_retrieval_event.clear()
        # self.sample_retrieval_future = self.tpe.submit(
        #     self.sample_retrieval_loop,
        #     stop_event=self.stop_sample_retrieval_event,
        # )
        if self.executor is None:
            raise RuntimeError("Executor for Eyelink node is not set")
        self.sample_retrieval_future = self.executor.create_task(
            self.sample_retrieval_loop,
            stop_event=self.stop_sample_retrieval_event,
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
        self.log("Starting recording")
        if self.recording:
            self.log("Already recording, skipping start", severity="WARN")
            return

        self.start_sample_retrieval()
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
        self.log("Stopping recording")
        if not self.recording:
            self.log("Not recording, skipping stop", severity="WARN")
            return

        self.stop_sample_retrieval()
        self.tracker.stopRecording()
        self.recording = False

    def close_data_file(self):
        """Closes the data file and transfers it to the local machine.

        This function will stop the recording (if it is running), close the
        data file on the EyeLink PC and transfer it to the local machine. It
        will then convert the EDF file to ASC format.
        """
        self.log("Closing data file")
        if self.recording:
            self.stop_recording()

        self.tracker.setOfflineMode()
        self.tracker.closeDataFile()

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
        return EyeAvailable(self.tracker.eyeAvailable())

    def left_eye_available(self) -> bool:
        """Check if the left eye is available."""
        return self.eye_available() in [
            EyeAvailable.LEFT_EYE,
            EyeAvailable.BINOCULAR,
        ]

    def right_eye_available(self) -> bool:
        """Check if the right eye is available."""
        return self.eye_available() in [
            EyeAvailable.RIGHT_EYE,
            EyeAvailable.BINOCULAR,
        ]

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

        assert left_sample is not None
        assert right_sample is not None

        # eye_used = self.eye_available()
        # if eye_used == EyeAvailable.LEFT_EYE:
        #     assert left_sample is not None
        #     assert right_sample is None
        # elif eye_used == EyeAvailable.RIGHT_EYE:
        #     assert left_sample is None
        #     assert right_sample is not None
        # elif eye_used == EyeAvailable.BINOCULAR:
        #     assert left_sample is not None
        #     assert right_sample is not None
        # elif eye_used == EyeAvailable.NO_EYE:
        #     assert left_sample is None
        #     assert right_sample is None
        #     self.log("No eye available", severity="WARN")
        #     return None
        # else:
        #     assert False, f"Unknown eye available value: {eye_used}"

        msg = EyelinkMsg()
        msg.header.stamp = timestamp.to_msg()
        msg.eyelink_time_ms = int(sample.getTime())
        msg.input = sample.getInput()

        msg.left_x, msg.left_y = left_sample.getRawPupil()
        msg.left_pupil = left_sample.getPupilSize()

        msg.right_x, msg.right_y = right_sample.getRawPupil()
        msg.right_pupil = right_sample.getPupilSize()

        return msg

    def sample_retrieval_loop(self, stop_event: threading.Event):
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
        try:
            wait_for_data_timeout_ms = int(
                self.get_parameter_wrapper("wait_for_data_timeout") * 1e3
            )
            # start_time = self.ros_time()
            while not stop_event.is_set():
                if not self.tracker.isConnected():
                    self.ros_sleep(1e-3)
                    continue
                try:
                    self.tracker.waitForData(wait_for_data_timeout_ms, 1, 0)
                except RuntimeError as e:
                    self.log(
                        f"No data from tracker with error: {e}",
                        severity="WARN",
                    )
                    continue

                sample: Sample | None = self.tracker.getNewestSample()
                if sample is not None and isinstance(sample, Sample):
                    timestamp = self.get_clock().now()
                    self.tracker.resetData()
                    msg = self.sample_to_msg(sample, timestamp)
                    with self.message_queue_lock:
                        self.message_queue.append(msg)
                    self.bag_writer.write(
                        "/eyelink/sample",
                        serialize_message(msg),  # type: ignore
                        timestamp.nanoseconds,
                    )

                # Sleep for a short period to avoid busy-waiting (necessary
                # for avoiding delayed execution in other threads)
                self.ros_sleep(1e-5)
        except (NotInitializedException, ROSSleepError):
            pass

    def get_last_messages(self) -> list[EyelinkMsg]:
        """Get the latest samples from the eyelink tracker.

        This function will return a copy of the contents of the sample queue
        as a list.
        """
        with self.message_queue_lock:
            return list(self.message_queue)

    ###########################################################################
    # Smooth pursuit
    ###########################################################################

    def get_smooth_pursuit(
        self, window: float, threshold: int, min_samples: int
    ) -> bool:
        """Check if the subject is smoothly pursuing.

        This function will check if the subject is smoothly pursuing by
        checking if the speed of the left and right eyes is below a threshold
        (the eye can only move smoothly if it is following a smoothly moving
        object, so we check if the speed remains below a threshold).

        Args:
            window: The window size in seconds.
            threshold: The threshold for the speed of the eyes.

        Returns:
            True if the subject is smoothly pursuing, False otherwise.
        """
        self.log("Checking for smooth pursuit", severity="DEBUG")

        messages = self.get_last_messages()

        num_samples = int(window * self.get_parameter_wrapper("sample_rate"))
        assert num_samples > 0 and num_samples <= len(messages)

        messages = messages[-num_samples:]

        # Track duration of smooth pursuit extraction, for logging purposes
        start_time = self.ros_time()

        # Extract eye data from samples
        eyelink_times = (
            np.array([message.eyelink_time_ms for message in messages])
            / 1000.0
        )  # Convert to seconds
        left_positions = np.array(
            [[message.left_x, message.left_y] for message in messages]
        )
        right_positions = np.array(
            [[message.right_x, message.right_y] for message in messages]
        )

        # Remove rows with missing data from the arrays
        valid_mask = (left_positions != MISSING_DATA).all(axis=1) and (
            right_positions != MISSING_DATA
        ).all(axis=1)
        eyelink_times = eyelink_times[valid_mask]
        left_positions = left_positions[valid_mask]
        right_positions = right_positions[valid_mask]

        # Check if there are enough samples for meaningful smooth pursuit
        # extraction
        if eyelink_times.shape[0] < min_samples:
            raise RuntimeError("Not enough valid samples")

        # TODO: Add smoothing and/or filtering to the positional data so as
        # to false negatives (e.g. if a spike of noise occurs for a single
        # sample, it is likely not a saccade/break in smooth pursuit and we
        # should ignore it).
        left_speed = np.linalg.norm(
            np.gradient(left_positions, eyelink_times, axis=0), axis=1
        )
        right_speed = np.linalg.norm(
            np.gradient(right_positions, eyelink_times, axis=0), axis=1
        )

        # Ensure that smooth pursuit is occuring by checking if the speeds of
        # the left and right eyes are below a threshold
        is_smoothly_pursuing = np.all(
            np.stack([left_speed, right_speed], axis=1) < threshold
        ).item()

        # Log the smooth pursuit status and statistics about the eye speed data
        # if self.get_logger().get_effective_level() <= logging.DEBUG:
        self.log(
            f"Time taken: {self.ros_time() - start_time}", severity="DEBUG"
        )
        if self.left_eye_available():
            left_speed_rounded = left_speed.round(2)
            # self.log(
            #     f"Left speed: {left_speed_rounded.tolist()}",
            #     severity="DEBUG",
            # )
            self.log(
                f"Left speed min: {left_speed_rounded.min()}, max: {left_speed_rounded.max()}, mean: {left_speed_rounded.mean()}",
                severity="DEBUG",
            )
        if self.right_eye_available():
            right_speed_rounded = right_speed.round(2)
            # self.log(
            #     f"Right speed: {right_speed_rounded.tolist()}",
            #     severity="DEBUG",
            # )
            self.log(
                f"Right speed min: {right_speed_rounded.min()}, max: {right_speed_rounded.max()}, mean: {right_speed_rounded.mean()}",
                severity="DEBUG",
            )

        return is_smoothly_pursuing

    ###########################################################################
    # ROS callbacks
    ###########################################################################

    # def log_callback(self):
    #     """Periodically log the smooth pursuit status and the newest valid sample."""
    #     self.log("Logging", severity="DEBUG")
    #     try:
    #         self.i += 1
    #         if self.i % 10 == 0:
    #             self.log(
    #                 f"Log interval: {(self.ros_time() - self.log_start_time):.5f} s",
    #                 severity="INFO",
    #             )
    #             self.log_start_time = self.ros_time()
    #             self.i = 0
    #     except AttributeError:
    #         self.i = 0
    #         self.log_start_time = self.ros_time()

    #     if self.get_parameter_wrapper("log_smooth_pursuit"):
    #         try:
    #             is_smoothly_pursuing = self.get_smooth_pursuit(
    #                 self.get_parameter_wrapper("log_smooth_pursuit_window"),
    #                 self.get_parameter_wrapper("log_smooth_pursuit_threshold"),
    #                 self.get_parameter_wrapper(
    #                     "log_smooth_pursuit_min_samples"
    #                 ),
    #             )
    #             self.log(f"Is smoothly pursuing: {is_smoothly_pursuing}")
    #         except RuntimeError as e:
    #             self.log(
    #                 f"Error getting smooth pursuit: {e}", severity="ERROR"
    #             )

    #     if self.get_parameter_wrapper("log_samples"):
    #         eye_data = self.get_last_eye_data()
    #         if len(eye_data) == 0:
    #             self.log(
    #                 "No samples in queue during periodic log", severity="WARN"
    #             )
    #             return
    #         for eye_datapoint in reversed(eye_data):
    #             if eye_datapoint is None:
    #                 continue
    #             timestamp, left_data, right_data = eye_datapoint

    #             self.log(f"Timestamp: {timestamp}")
    #             if left_data is not None:
    #                 self.log(
    #                     f"Left eye x: {left_data[0]}, y: {left_data[1]}, diameter: {left_data[2]}"
    #                 )
    #             if right_data is not None:
    #                 self.log(
    #                     f"Right eye x: {right_data[0]}, y: {right_data[1]}, diameter: {right_data[2]}"
    #                 )
    #             return
    #         self.log("Found no adequate samples", severity="WARN")

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

    def smooth_pursuit_goal_callback(
        self, goal: EyelinkSmoothPursuit.Goal
    ) -> GoalResponse:
        with self.goal_callback_lock:
            if self.goal_ongoing:
                self.log(
                    "Cannot accept new goal, previous goal not finished",
                    severity="WARN",
                )
                return GoalResponse.REJECT
            elif Duration.from_msg(goal.window) > Duration(
                seconds=self.max_window
            ):
                self.log(
                    f"Window too long, must be less than {self.max_window} s",
                    severity="WARN",
                )
                return GoalResponse.REJECT
            else:
                self.goal_ongoing = True
                return GoalResponse.ACCEPT

    def smooth_pursuit_cancel_callback(self, _) -> CancelResponse:
        with self.goal_callback_lock:
            if self.goal_ongoing:
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
        """Flic response time action callback."""
        try:
            self.log("Starting smooth pursuit")

            # Get the smooth pursuit parameters from the goal
            window = (
                Duration.from_msg(goal_handle.request.window).nanoseconds / 1e9
            )
            threshold = goal_handle.request.threshold
            min_samples = goal_handle.request.min_samples
            last_time = self.get_clock().now()

            # Get the initial smooth pursuit status
            last_feedback = EyelinkSmoothPursuit.Feedback()
            last_feedback.smooth_pursuit_started = self.get_smooth_pursuit(
                window, threshold, min_samples
            )
            last_feedback.duration = Duration(seconds=0).to_msg()
            goal_handle.publish_feedback(last_feedback)

            # Loop until the goal is cancelled
            while not goal_handle.is_cancel_requested:
                self.ros_sleep(0.1)
                smooth_pursuit = self.get_smooth_pursuit(
                    window, threshold, min_samples
                )
                if smooth_pursuit != last_feedback.smooth_pursuit_started:
                    cur_time = self.get_clock().now()
                    feedback = EyelinkSmoothPursuit.Feedback()
                    feedback.smooth_pursuit_started = smooth_pursuit
                    duration = cur_time - last_time
                    feedback.duration = duration.to_msg()
                    goal_handle.publish_feedback(feedback)

                    if smooth_pursuit:
                        self.log(
                            f"Smooth pursuit started after {duration.nanoseconds / 1e9:.2f} s"
                        )
                    else:
                        self.log(
                            f"Smooth pursuit ended after {duration.nanoseconds / 1e9:.2f} s"
                        )
                    last_time = cur_time
                    last_feedback = feedback

            goal_handle.canceled()
            return EyelinkSmoothPursuit.Result()
        finally:
            with self.goal_callback_lock:
                self.goal_ongoing = False

    ###########################################################################
    # Node lifecycle
    ###########################################################################

    def destroy_node(self):
        """Destroy the node.

        This function will stop the sample retrieval loop, stop the recording,
        close the data file, and close the tracker.
        """

        try:
            self.close_data_file()
        except Exception as e:
            self.log(
                f"Error closing data file: {type(e).__name__}: {e}",
                severity="ERROR",
            )

        try:
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

        self.log("Closing tracker")
        self.tracker.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
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
