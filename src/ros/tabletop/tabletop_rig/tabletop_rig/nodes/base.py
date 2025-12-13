import asyncio
from typing import Any, Optional, cast

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from rclpy.duration import Duration
from rclpy.exceptions import (
    ParameterAlreadyDeclaredException,
    ParameterNotDeclaredException,
)
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node

from tabletop_py.utils.common import yaml_dump_string
from tabletop_rig.exceptions import (
    ROSSleepError,
    ServiceCallTimeoutError,
    ServiceCallUnsuccessfulError,
)
from tabletop_rig.utils.logging import LoggerMixin, msg_to_dict
from tabletop_rig.utils.ros import SrvType, SrvTypeRequest, SrvTypeResponse


class BaseNode(Node, LoggerMixin):
    """
    Base class for all nodes.

    This class extends the Node class with common functionality, including
    parameter declaration, logging, and service calls.
    """

    # Optional parameters with default values
    default_params: dict[str, Any] = {
        "default_service_wait_timeout": 5.0,
        "default_service_call_timeout": 2.0,
    }
    # Required parameters
    required_params: set[str] = set()

    def __init__(
        self,
        *args,
        **kwargs,
    ):
        Node.__init__(self, *args, **kwargs)
        self._check_parameters()
        self._declare_default_parameters()
        # self.log_parameters(severity="DEBUG")

    def _check_parameters(self):
        """
        Check if there is any intersection between required and default
        parameters or if any required parameter is not declared. If so,
        raise an error.
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
            except ParameterNotDeclaredException:
                msg = f"Required parameter {name} not declared for {self.get_name()} node"
                self.log(msg, severity="ERROR")
                raise ParameterNotDeclaredException(msg)

    def _declare_default_parameters(self):
        """
        Declare the default parameters, which are used if no overrides are
        provided.
        """
        for name, value in self.default_params.items():
            try:
                self.declare_parameter(name, value)
            except ParameterAlreadyDeclaredException:
                self.log(
                    f"Parameter {name} already declared, using override",
                    severity="WARN",
                )

    def log_parameters(
        self,
        prefix: str = "",
        severity: str | LoggingSeverity = "INFO",
        **kwargs: Any,
    ) -> bool:
        """
        Log all parameters with the given prefix.
        """
        if not isinstance(severity, LoggingSeverity):
            severity = LoggingSeverity[severity]

        if severity >= self.log_level:
            params = self.get_nested_parameters(prefix)
            string = yaml_dump_string(params, width=self.param("yaml_width"))
            if prefix:
                string = f"Parameters with prefix {prefix}:\n{string}"
            success = self.log(string, severity=severity, **kwargs)
            assert success
            return True
        else:
            return False

    def get_nested_parameters(self, prefix: str = "") -> dict:
        """Get a nested dictionary of parameters with the given prefix.

        Retrieves all parameters from a ROS2 node and structures them into a
        nested dictionary. Namespaces are represented as nested dictionaries.
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

    def param(self, name: str) -> Any:
        """Get a parameter from the node."""
        try:
            value = self.get_parameter(name).value
            return value if value != "null" else None
        except ParameterNotDeclaredException:
            return self.get_nested_parameters(name)

    def ros_time(self) -> float:
        """Get the current time in seconds from the ROS2 clock."""
        return float(self.get_clock().now().nanoseconds) / 1e9

    def ros_sleep(self, seconds: float):
        """Sleep for the given number of seconds."""
        if not self.get_clock().sleep_for(Duration(seconds=seconds)):
            raise ROSSleepError("ROS2 clock did not sleep correctly")

    def _create_client(
        self,
        srv_type: Optional[type] = None,
        srv_name: Optional[str] = None,
    ) -> Client:
        """
        Create a client for a service or return the provided client if it
        is not None.
        """
        if srv_type is None or srv_name is None:
            raise ValueError(
                "srv_type and srv_name must be provided if service_client is not provided"
            )

        srv_client = self.create_client(
            srv_type,
            srv_name,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        return srv_client

    def validate_service_client(
        self,
        service_client: Client,
        srv_type: type | None,
        srv_name: str | None,
    ):
        """Validate the service client."""
        if (srv_type is not None and srv_type != service_client.srv_type) or (
            srv_name is not None and srv_name != service_client.service_name
        ):
            raise ValueError(
                "srv_type and srv_name must be None if service_client is provided, or they must match the service client"
            )

    def validate_service_response(
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
            error_msg = (
                f"{service_client.service_name} service call timed out!"
            )
            raise ServiceCallTimeoutError(error_msg)
        elif hasattr(response, "success") and not response.success:  # type: ignore
            error_msg = f"{service_client.service_name} service call returned unsuccessfully with response: {msg_to_dict(response)}"
            raise ServiceCallUnsuccessfulError(error_msg)

    def wait_for_service_blocking(
        self,
        srv_type: Optional[type] = None,
        srv_name: Optional[str] = None,
        *,
        service_client: Optional[Client] = None,
        timeout: Optional[float] = None,
    ):
        """Wait for a service to be available.

        Args:
            srv_type: The type of the service.
            srv_name: The name of the service.
            srv_client: The service client to use.
            timeout_sec: The timeout in seconds.

        Raises:
            ServiceCallError: If the service call fails.
            ServiceCallUnsuccessfulError: If the service call is unsuccessful.
        """
        timeout = (
            timeout
            if timeout is not None
            else self.param("default_service_wait_timeout")
        )

        # If the service client is not provided, create a new one and destroy
        # it after the service call
        if service_client is None:
            service_client = self._create_client(srv_type, srv_name)
            destroy_service_client = True
        else:
            self.validate_service_client(service_client, srv_type, srv_name)
            destroy_service_client = False

        try:
            self.log(
                f"Waiting for {srv_name} service to be available...",
                severity="DEBUG",
            )
            if not service_client.wait_for_service(timeout_sec=timeout):
                error_msg = f"{srv_name} not available!"
                self.log(error_msg, severity="ERROR")
                raise ServiceCallTimeoutError(error_msg)
            self.log(f"{srv_name} service is available", severity="DEBUG")
        finally:
            # Destroy the service client if it was created by this function
            if destroy_service_client:
                self.destroy_client(service_client)

    async def wait_for_service_async(self, *args, **kwargs):
        """Wait for a service to be available (asynchronous)."""
        await asyncio.to_thread(
            self.wait_for_service_blocking, *args, **kwargs
        )

    def service_call_blocking(
        self,
        srv_request: SrvTypeRequest,
        srv_type: Optional[type[SrvType]] = None,
        srv_name: Optional[str] = None,
        *,
        srv_client: Optional[Client] = None,
        timeout: Optional[float] = None,
    ) -> SrvTypeResponse:
        """Call a service synchronously, returning the response.

        Args:
            srv_request: The request message for the service.
            srv_type: The type of the service.
            srv_name: The name of the service.
            service_client: The service client to use.
            timeout: The timeout in seconds.

        Returns:
            The response from the service.

        Raises:
            ServiceCallError: If the service call fails.
            ServiceCallUnsuccessfulError: If the service call is unsuccessful.
        """
        timeout = (
            timeout
            if timeout is not None
            else self.param("default_service_call_timeout")
        )

        # If the service client is not provided, create a new one and destroy
        # it after the service call
        if srv_client is None:
            srv_client = self._create_client(srv_type, srv_name)
            destroy_service_client = True
        else:
            self.validate_service_client(srv_client, srv_type, srv_name)
            destroy_service_client = False

        try:
            # self.log(
            #     f"Calling {service_client.service_name} service...",
            #     severity="DEBUG",
            # )
            response = srv_client.call(srv_request, timeout_sec=timeout)
            # self.log(
            #     f"Service call to {service_client.service_name} finished with response:",
            #     severity="DEBUG",
            # )
            # self.log_ros_msg(
            #     response,
            #     title=f"{service_client.service_name} response",
            #     severity="DEBUG",
            # )
            self.validate_service_response(response, srv_client)
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
        """Call a service asynchronously, returning a future and the service client.

        Args:
            srv_request: The request message for the service.
            srv_type: The type of the service.
            srv_name: The name of the service.
            srv_client: The service client to use.
            timeout: The timeout in seconds.

        Returns:
            The response from the service.

        Raises:
            ServiceCallError: If the service call fails.
            ServiceCallUnsuccessfulError: If the service call is unsuccessful.
        """
        timeout = (
            timeout
            if timeout is not None
            else self.param("default_service_call_timeout")
        )

        # If the service client is not provided, create a new one and
        # destroy it after the service call
        if srv_client is None:
            srv_client = self._create_client(srv_type, srv_name)
            destroy_service_client = True
        else:
            self.validate_service_client(srv_client, srv_type, srv_name)
            destroy_service_client = False

        try:
            # self.log(
            #     f"Calling {service_client.service_name} service asynchronously...",
            #     severity="DEBUG",
            # )
            future = srv_client.call_async(srv_request)

            async with asyncio.timeout(timeout):
                response = await future
            # self.log(
            #     f"Service call to {service_client.service_name} finished with response:",
            #     severity="DEBUG",
            # )
            # self.log_ros_msg(
            #     response,
            #     title=f"{service_client.service_name} response",
            #     severity="DEBUG",
            # )
            self.validate_service_response(response, srv_client)
            return cast(SrvTypeResponse, response)
        except TimeoutError:
            raise ServiceCallTimeoutError(
                f"{srv_client.service_name} service call timed out!"
            )
        finally:
            if destroy_service_client:
                self.destroy_client(srv_client)
