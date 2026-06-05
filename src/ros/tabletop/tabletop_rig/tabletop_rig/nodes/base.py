"""Base node class providing common ROS2 functionality.

This module provides the BaseNode class which extends rclpy's Node with
common functionality used across all tabletop_rig nodes:

- Parameter declaration with defaults and validation
- Nested parameter access (dot-separated names)
- Structured logging via LoggerMixin
- Synchronous and asynchronous service calls
- Time and sleep utilities

All custom nodes in this package should inherit from BaseNode rather
than directly from rclpy.Node.

Example:
    class MyNode(BaseNode):
        default_params = BaseNode.default_params | {
            "my_param": 1.0,
        }
        required_params = {"required_param"}

        def __init__(self):
            super().__init__("my_node")
            value = self.param("my_param")
"""

import asyncio
from collections.abc import Callable
from copy import deepcopy
from typing import TYPE_CHECKING, Any, Optional, cast

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.callback_groups import CallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from rclpy.duration import Duration
from rclpy.exceptions import (
    ParameterAlreadyDeclaredException,
    ParameterNotDeclaredException,
)
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSProfile,
    qos_profile_default,
    qos_profile_services_default,
)
from rclpy.service import Service
from rclpy.service_introspection import ServiceIntrospectionState

from tabletop_py.utils.common import yaml_dump_string
from tabletop_rig.exceptions import (
    ActionGoalNotAcceptedError,
    ActionResultUnsuccessfulError,
    ActionServerWaitTimeoutError,
    ROSSleepError,
    ServiceCallTimeoutError,
    ServiceCallUnsuccessfulError,
)
from tabletop_rig.utils.logging import LoggerMixin, msg_to_dict
from tabletop_rig.utils.ros import SrvType, SrvTypeRequest, SrvTypeResponse

if TYPE_CHECKING:
    from tabletop_rig.interfaces.base import BaseInterface

_CANCEL_WRAPPED_FUTURES = False


def flatten_dict(d: Any, prefix: str = "", sep: str = ".") -> dict:
    if not isinstance(d, dict):
        return {prefix: d}

    if prefix != "":
        prefix = f"{prefix}{sep}"

    result = {}
    for k, v in d.items():
        result.update(flatten_dict(v, f"{prefix}{k}", sep))

    return result


def _get_loop(fut: asyncio.Future):
    # Tries to call Future.get_loop() if it's available.
    # Otherwise fallbacks to using the old '_loop' property.
    try:
        get_loop = fut.get_loop
    except AttributeError:
        pass
    else:
        return get_loop()
    return fut._loop


def _copy_future_state(source: rclpy.Future, destination: asyncio.Future):
    assert source.done() or source.cancelled()
    if destination.cancelled():
        return
    assert not destination.done()
    if source.cancelled():
        destination.cancel()
    else:
        exception = source.exception()
        if exception is not None:
            destination.set_exception(exception)
        else:
            result = source.result()
            destination.set_result(result)


def _chain_future(source: rclpy.Future, destination: asyncio.Future):
    """Chain two futures so that when one completes, so does the other.

    The result (or exception) of source will be copied to destination.
    If destination is cancelled, source gets cancelled too.
    Compatible with both asyncio.Future and concurrent.futures.Future.
    """
    if not isinstance(source, rclpy.Future):
        raise TypeError("An rclpy future is required for source argument")
    if not asyncio.isfuture(destination):
        raise TypeError("A future is required for destination argument")

    dest_loop = _get_loop(destination)
    assert dest_loop is not None

    def _set_state(src):
        if destination.cancelled() and dest_loop.is_closed():
            return

        source_loop = None
        try:
            source_loop = asyncio.get_running_loop()
        except RuntimeError:
            pass

        if dest_loop is source_loop:
            _copy_future_state(src, destination)
        else:
            if dest_loop.is_closed():
                return
            dest_loop.call_soon_threadsafe(
                _copy_future_state, src, destination
            )

    def _check_cancel(dest):
        if dest.cancelled():
            source.cancel()

    def _check_remove_callback(dest):
        if dest.cancelled():
            source.remove_done_callback(_set_state)

    if _CANCEL_WRAPPED_FUTURES:
        destination.add_done_callback(_check_cancel)
    else:
        destination.add_done_callback(_check_remove_callback)

    source.add_done_callback(_set_state)


def wrap_rclpy_future(future: rclpy.Future, *, loop=None):
    """Wrap concurrent.futures.Future object."""
    assert isinstance(future, rclpy.Future), (
        f"rclpy.task.Future is expected, got {future!r}"
    )
    if loop is None:
        loop = asyncio.get_event_loop()
    new_future = loop.create_future()
    _chain_future(future, new_future)
    return new_future


class AIOActionClient(ActionClient):
    def wait_for_server(self, timeout_sec=None):
        ready = super().wait_for_server(timeout_sec)
        if not ready:
            raise ActionServerWaitTimeoutError(
                f"Waiting for {self._action_name} action server timed out after {timeout_sec}"
            )

    async def wait_for_server_async(self, timeout_sec: Optional[float] = None):
        """Async version of wait_for_server.

        Runs the blocking wait in a thread pool to avoid blocking
        the asyncio event loop.
        """
        return await asyncio.to_thread(
            self.wait_for_server, timeout_sec=timeout_sec
        )

    async def send_goal_async(  # type: ignore
        self, goal, feedback_callback=None, goal_uuid=None
    ) -> ClientGoalHandle:
        goal_handle = await wrap_rclpy_future(
            super().send_goal_async(
                goal=goal,
                feedback_callback=feedback_callback,
                goal_uuid=goal_uuid,
            )
        )
        if not goal_handle.accepted:
            raise ActionGoalNotAcceptedError(
                f"{self._action_name} action goal request not accepted"
            )

        return goal_handle

    async def get_result_async(self, goal_handle: ClientGoalHandle):
        try:
            response = await wrap_rclpy_future(goal_handle.get_result_async())
        finally:
            if goal_handle.status not in (
                GoalStatus.STATUS_CANCELING,
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_ABORTED,
            ):
                goal_handle.cancel_goal_async()

        if response.status != GoalStatus.STATUS_SUCCEEDED:
            raise ActionResultUnsuccessfulError(self._action_name, response)

        return response.result


class BaseNode(Node, LoggerMixin):
    """Base class for all tabletop_rig ROS2 nodes.

    This class extends rclpy.Node with common functionality including
    parameter declaration, structured logging, and service call utilities.
    All nodes in this package should inherit from BaseNode.

    Subclasses can define required and default parameters by overriding
    the class attributes. Parameter validation occurs during __init__.

    Attributes:
        default_params: Dict of parameter names to default values. These
            are declared automatically if not overridden by launch config.
        required_params: Set of parameter names that must be declared before
            node initialization (typically via launch file).

    Example:
        class MyNode(BaseNode):
            default_params = BaseNode.default_params | {"rate": 10.0}
            required_params = {"device_name"}

            def __init__(self):
                super().__init__("my_node")
    """

    default_params: dict[str, Any] = {
        "default_service_wait_timeout": 5.0,
        "default_service_call_timeout": 5.0,
        "default_node_wait_timeout": 5.0,
        "enable_service_introspection": True,
    }
    required_params: set[str] = set()

    def __init__(
        self,
        *args,
        **kwargs,
    ):
        """Initialize the base node.

        Args:
            *args: Positional arguments passed to rclpy.Node.__init__.
            **kwargs: Keyword arguments passed to rclpy.Node.__init__.

        Raises:
            ValueError: If required and default parameters intersect.
            ParameterNotDeclaredException: If a required parameter is missing.
        """
        self._initialized = False
        Node.__init__(self, *args, **kwargs)
        self._check_parameters()
        self._declare_default_parameters()
        self._interfaces: list["BaseInterface"] = []
        self._initialized = True

    def _check_parameters(self):
        """Validate parameter configuration.

        Ensures required and default parameters don't overlap and that
        all required parameters have been declared (via launch file).

        Raises:
            ValueError: If required and default parameters intersect.
            ParameterNotDeclaredException: If a required parameter is missing.
        """
        # Check for intersections
        if self.required_params & self.default_params.keys():
            raise ValueError(
                "Required parameters cannot intersect with default parameters"
            )

        # Check for required parameters
        for name in self.required_params:
            try:
                self.param(name)
            except ParameterNotDeclaredException as e:
                raise ParameterNotDeclaredException(
                    f"Required parameter {name} not declared for {self.get_name()} node"
                ) from e

    def _declare_default_parameters(self):
        """Declare default parameters if not already declared.

        Iterates through default_params and declares each parameter
        with its default value. If a parameter was already declared
        (e.g., via launch file override), logs a warning and skips.
        """
        for name, value in self.default_params.items():
            try:
                self.declare_parameter(name, value)
            except ParameterAlreadyDeclaredException:
                self.log(
                    f"Parameter {name} already declared, using override",
                    severity="WARN",
                )

    def register_interface(self, interface: "BaseInterface"):
        """Register a BaseInterface to the class for proper cleanup"""
        self._interfaces.append(interface)

    def log_parameters(
        self,
        prefix: str = "",
        severity: str | LoggingSeverity = "INFO",
    ) -> bool:
        """Log all parameters matching a prefix in YAML format.

        Args:
            prefix: Parameter name prefix filter. Empty string matches all.
            severity: Logging severity level (string or LoggingSeverity).
            **kwargs: Additional arguments passed to self.log().

        Returns:
            True if parameters were logged, False if severity below threshold.

        Raises:
            ParameterNotDeclaredException: If no parameters match prefix.
        """
        if not isinstance(severity, LoggingSeverity):
            severity = LoggingSeverity[severity]

        if severity >= self.log_level:
            params = self.get_nested_parameters(prefix)
            string = yaml_dump_string(params, width=self.param("yaml_width"))
            if prefix:
                string = f"Parameters with prefix {prefix}:\n{string}"
            success = self.log(string, severity=severity)
            assert success
            return True
        else:
            return False

    def get_nested_parameters(self, prefix: str = "") -> dict[str, Any]:
        """Get parameters as a nested dictionary structure.

        Retrieves all parameters matching the prefix and structures them
        into nested dicts based on dot-separated names. For example,
        parameters "foo.bar" and "foo.baz" become {"foo": {"bar": ..., "baz": ...}}.

        Args:
            prefix: Parameter name prefix filter.

        Returns:
            Nested dictionary of parameter values.

        Raises:
            ParameterNotDeclaredException: If no parameters match prefix.
        """
        params = self.get_parameters_by_prefix(prefix)
        if len(params) == 0:
            raise ParameterNotDeclaredException(
                f"No parameters found for prefix {prefix}"
            )
        nested_params = {}

        for name, param in params.items():
            value = param.value  # type: ignore
            keys = name.split(".")
            current_level = nested_params

            for key in keys[:-1]:
                current_level = current_level.setdefault(key, {})

            current_level[keys[-1]] = value if value != "null" else None

        return nested_params

    def set_or_declare_nested_parameters(
        self, nested_parameters: dict[str, Any], prefix: str = ""
    ) -> None:
        flattened = flatten_dict(nested_parameters, prefix=prefix)

        params: list[Parameter] = []
        for k, v in flattened.items():
            if self.has_parameter(k):
                params.append(Parameter(name=k, value=v))
            else:
                self.declare_parameter(k, v)

        self.set_parameters(params)

    def param(self, name: str) -> Any:
        """Get a parameter value by name.

        Attempts to get a single parameter first. If the name represents
        a namespace (prefix), returns a nested dictionary of all parameters
        under that prefix.

        Args:
            name: Parameter name (may be dot-separated for nested access).

        Returns:
            Parameter value, or nested dict if name is a prefix.
            Returns None for parameters set to "null" string.
        """
        try:
            value = self.get_parameter(name).value
            return deepcopy(value) if value != "null" else None
        except ParameterNotDeclaredException:
            return deepcopy(self.get_nested_parameters(name))

    def ros_time(self) -> float:
        """Get current time in seconds from the ROS2 clock.

        Returns:
            Current time in seconds (floating point).
        """
        return float(self.get_clock().now().nanoseconds) / 1e9

    def ros_sleep(self, seconds: float):
        """Sleep for the specified duration using ROS2 clock.

        Args:
            seconds: Sleep duration in seconds.

        Raises:
            ROSSleepError: If the ROS2 clock fails to sleep correctly.
        """
        if not self.get_clock().sleep_for(Duration(seconds=seconds)):
            raise ROSSleepError("ROS2 clock did not sleep correctly")

    def wait_for_node_blocking(
        self, fully_qualified_node_name: str, timeout: Optional[float] = None
    ) -> bool:
        if timeout is None:
            timeout = cast(float, self.param("default_node_wait_timeout"))

        return super().wait_for_node(fully_qualified_node_name, timeout)

    async def wait_for_node_async(
        self, fully_qualified_node_name: str, timeout: Optional[float] = None
    ) -> bool:
        """Async version of wait_for_node.

        Runs the blocking wait in a thread pool to avoid blocking
        the asyncio event loop.

        Args:
            fully_qualified_node_name: Fully qualified name of the node to
                wait for.
            timeout: Seconds to wait for the node to be present.
                If negative, the function won't timeout.
        Returns
            True if the node was found, False if timeout.
        """
        if timeout is None:
            timeout = cast(float, self.param("default_node_wait_timeout"))

        return await asyncio.to_thread(
            self.wait_for_node, fully_qualified_node_name, timeout
        )

    def create_client(
        self,
        srv_type,
        srv_name: str,
        *,
        qos_profile: QoSProfile = qos_profile_services_default,
        callback_group: Optional[CallbackGroup] = None,
        enable_introspection: bool = True,
    ) -> Client:
        srv_client = super().create_client(
            srv_type,
            srv_name,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        if (
            self._initialized
            and enable_introspection
            and self.param("enable_service_introspection")
        ):
            srv_client.configure_introspection(
                self.get_clock(),
                qos_profile_default,
                ServiceIntrospectionState.CONTENTS,
            )

        return srv_client

    def create_service(
        self,
        srv_type,
        srv_name: str,
        callback: Callable[[SrvTypeRequest, SrvTypeResponse], SrvTypeResponse],
        *,
        qos_profile: QoSProfile = qos_profile_services_default,
        callback_group: Optional[CallbackGroup] = None,
    ) -> Service:
        service = super().create_service(
            srv_type,
            srv_name,
            callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        if self._initialized and self.param("enable_service_introspection"):
            service.configure_introspection(
                self.get_clock(),
                qos_profile_default,
                ServiceIntrospectionState.CONTENTS,
            )

        return service

    def _validate_service_client(
        self,
        service_client: Client,
        srv_type: type | None,
        srv_name: str | None,
    ):
        """Validate that a service client matches expected type and name.

        Used to ensure a provided client is compatible with the intended
        service when srv_type or srv_name are also provided.

        Args:
            service_client: The client to validate.
            srv_type: Expected service type, or None to skip check.
            srv_name: Expected service name, or None to skip check.

        Raises:
            ValueError: If client doesn't match expected type or name.
        """
        if (srv_type is not None and srv_type != service_client.srv_type) or (
            srv_name is not None and srv_name != service_client.service_name
        ):
            raise ValueError(
                "srv_type and srv_name must be None if service_client is provided, or they must match the service client"
            )

    def _validate_service_response(
        self,
        response: SrvTypeResponse | None,
        service_client: Client,
    ) -> None:
        """Validate the response from a service call.

        Args:
            response: The response from a service call.
            service_client: The client that made the service call.

        Returns:
            The response from the service call.

        Raises:
            ServiceCallTimeoutError: If the service call timed out.
            ServiceCallUnsuccessfulError: If the service call returned with a failure status.
        """
        if response is None:
            raise ServiceCallTimeoutError(
                f"{service_client.service_name} service call timed out!"
            )
        elif hasattr(response, "success") and not response.success:  # type: ignore
            raise ServiceCallUnsuccessfulError(
                f"{service_client.service_name} service call returned unsuccessfully with response: {msg_to_dict(response)}"
            )

    def wait_for_service_blocking(
        self, srv_client: Client, timeout: Optional[float] = None
    ) -> bool:
        if timeout is None:
            timeout = cast(float, self.param("default_service_wait_timeout"))

        return srv_client.wait_for_service(timeout)

    async def wait_for_service_async(
        self, srv_client: Client, timeout: Optional[float] = None
    ) -> bool:
        if timeout is None:
            timeout = cast(float, self.param("default_service_wait_timeout"))

        return await asyncio.to_thread(srv_client.wait_for_service, timeout)

    def service_call_blocking(
        self,
        srv_request: SrvTypeRequest,
        srv_type: Optional[type[SrvType]] = None,
        srv_name: Optional[str] = None,
        *,
        srv_client: Optional[Client] = None,
        timeout: Optional[float] = None,
    ) -> SrvTypeResponse:
        """Make a synchronous (blocking) service call.

        Creates a temporary client if srv_client is not provided. Validates
        the response and raises exceptions for timeouts or failures.

        Args:
            srv_request: The request message to send.
            srv_type: Service type class. Required if srv_client is None.
            srv_name: Service name. Required if srv_client is None.
            srv_client: Existing client to reuse.
            timeout: Call timeout in seconds. Defaults to parameter
                'default_service_call_timeout'.

        Returns:
            The service response message.

        Raises:
            ServiceCallTimeoutError: If the call times out.
            ServiceCallUnsuccessfulError: If response.success is False.
        """
        timeout = (
            timeout
            if timeout is not None
            else self.param("default_service_call_timeout")
        )

        # If the service client is not provided, create a new one and destroy
        # it after the service call
        if srv_client is None:
            if srv_type is None or srv_name is None:
                raise ValueError(
                    "srv_type and srv_name must be provided if service_client is not provided"
                )
            srv_client = self.create_client(
                srv_type,
                srv_name,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
            destroy_service_client = True
        else:
            self._validate_service_client(srv_client, srv_type, srv_name)
            destroy_service_client = False

        if not srv_client.service_is_ready():
            if not self.wait_for_service_blocking(srv_client):
                raise ServiceCallTimeoutError(
                    f"Wait for '{srv_client.service_name}' service timed out!"
                )

        try:
            response = srv_client.call(srv_request, timeout_sec=timeout)
            self._validate_service_response(response, srv_client)
            return cast(SrvTypeResponse, response)
        finally:
            # Destroy the service client if it was created by this function
            if destroy_service_client:
                self.destroy_client(srv_client)

    async def service_call_async(
        self,
        srv_request: SrvTypeRequest,
        srv_type: Optional[type[SrvType]] = None,
        srv_name: Optional[str] = None,
        *,
        srv_client: Optional[Client] = None,
        timeout: Optional[float] = None,
    ) -> SrvTypeResponse:
        """Make an asynchronous service call.

        Non-blocking version of service_call_blocking. Uses asyncio.timeout
        to enforce the timeout without blocking the event loop.

        Args:
            srv_request: The request message to send.
            srv_type: Service type class. Required if srv_client is None.
            srv_name: Service name. Required if srv_client is None.
            srv_client: Existing client to reuse.
            timeout: Call timeout in seconds. Defaults to parameter
                'default_service_call_timeout'.

        Returns:
            The service response message.

        Raises:
            ServiceCallTimeoutError: If the call times out.
            ServiceCallUnsuccessfulError: If response.success is False.
        """
        timeout = (
            timeout
            if timeout is not None
            else self.param("default_service_call_timeout")
        )

        # If the service client is not provided, create a new one and
        # destroy it after the service call
        if srv_client is None:
            if srv_type is None or srv_name is None:
                raise ValueError(
                    "srv_type and srv_name must be provided if service_client is not provided"
                )
            srv_client = self.create_client(
                srv_type,
                srv_name,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
            destroy_service_client = True
        else:
            self._validate_service_client(srv_client, srv_type, srv_name)
            destroy_service_client = False

        if not srv_client.service_is_ready():
            ready = await self.wait_for_service_async(srv_client)
            if not ready:
                raise ServiceCallTimeoutError(
                    f"Wait for '{srv_client.service_name}' service timed out!"
                )

        try:
            future = wrap_rclpy_future(srv_client.call_async(srv_request))
            async with asyncio.timeout(timeout):
                response = await future
            self._validate_service_response(response, srv_client)
            return cast(SrvTypeResponse, response)
        except TimeoutError:
            raise ServiceCallTimeoutError(
                f"{srv_client.service_name} service call timed out!"
            )
        finally:
            if destroy_service_client:
                self.destroy_client(srv_client)

    def destroy_node(self):
        for interface in reversed(self._interfaces):
            interface.destroy_interface()
        Node.destroy_node(self)
