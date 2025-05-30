import asyncio
from typing import Any, Optional

import rclpy
import yaml
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from rclpy.duration import Duration
from rclpy.exceptions import (
    ParameterAlreadyDeclaredException,
    ParameterNotDeclaredException,
)
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from tabletop_utils.common import BracketedListDumper
from tabletop_utils.ros import (
    SrvType,
    SrvTypeRequest,
    SrvTypeResponse,
    msg_to_dict,
    validate_service_response,
)

# from logging import DEBUG, INFO, WARN, ERROR, FATAL
# Move this to a constants file
DEFAULT_LOG_SEVERITY = "INFO"


class BaseNode(Node):
    """
    Base class for all nodes.

    This class extends the Node class with common functionality, including
    parameter declaration, logging, and service calls.
    """

    # Optional parameters with default values
    default_params: dict[str, Any] = {
        "default_service_wait_timeout": 5.0,
        "default_service_call_timeout": 2.0,
        "yaml_width": 120,
    }
    # Required parameters
    required_params: set[str] = set()

    def __init__(
        self,
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self._check_parameters()
        self._declare_default_parameters()
        self.log_params(severity="DEBUG")

    def log(
        self,
        message: Any,
        severity: str = DEFAULT_LOG_SEVERITY,
        **kwargs,
    ):
        """
        Log a message with the given severity.
        """
        kwargs["throttle_duration_sec"] = None
        if rclpy.ok():
            match severity:
                case "DEBUG":
                    self.get_logger().debug(message, **kwargs)
                case "INFO":
                    self.get_logger().info(message, **kwargs)
                case "WARN":
                    self.get_logger().warning(message, **kwargs)
                case "ERROR":
                    self.get_logger().error(message, **kwargs)
                case "FATAL":
                    self.get_logger().fatal(message, **kwargs)
                case _:
                    raise ValueError(f"Invalid severity: {severity}")
        else:
            if (
                LoggingSeverity[severity]
                >= self.get_logger().get_effective_level()
            ):
                print(f"{severity}: {message}")

    @property
    def log_level(self) -> LoggingSeverity:
        """Get the log severity."""
        return self.get_logger().get_effective_level()

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
                self.get_parameter_wrapper(name)
            except ParameterNotDeclaredException:
                msg = (
                    f"Required parameter {name} not declared "
                    f"for {self.get_name()} node"
                )
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

    def log_params(
        self, prefix: str = "", severity: str = DEFAULT_LOG_SEVERITY
    ):
        """
        Log all parameters with the given prefix.
        """
        if self.log_level < LoggingSeverity[severity]:
            return

        prefix = (
            f"{prefix}." if prefix and not prefix.endswith(".") else prefix
        )
        params = self.get_parameters_by_prefix(prefix)
        for param in params.values():
            self.log(f"{param.name}: {param.value}", severity=severity)  # type: ignore

    def log_ros_msg(
        self,
        msg: Any,
        title: Optional[str] = None,
        severity: str = DEFAULT_LOG_SEVERITY,
    ):
        """Log a ROS message as a YAML string."""
        if self.log_level < LoggingSeverity[severity]:
            return

        string = yaml.dump(
            msg_to_dict(msg),
            Dumper=BracketedListDumper,
            width=self.get_parameter_wrapper("yaml_width"),
        )
        if title is not None:
            string = f"{title}\n{string}"
        self.log(string, severity=severity)

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

    def get_parameter_wrapper(self, name: str) -> Any:
        """Get a parameter from the node."""
        try:
            value = super().get_parameter(name).value
            return value if value != "null" else None
        except ParameterNotDeclaredException:
            return self.get_nested_parameters(name)

    def time(self) -> float:
        """Get the current time in seconds from the ROS2 clock."""
        return float(self.get_clock().now().nanoseconds) / 1e9

    def sleep(self, seconds: float):
        """Sleep for the given number of seconds."""
        if not self.get_clock().sleep_for(Duration(seconds=seconds)):
            raise RuntimeError("ROS2 clock did not sleep correctly")

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
            msg = "srv_type and srv_name must be provided if service_client is not provided"
            self.log(msg, severity="ERROR")
            raise ValueError(msg)

        service_client = self.create_client(
            srv_type,
            srv_name,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        return service_client

    def wait_for_service(
        self,
        srv_type: Optional[type] = None,
        srv_name: Optional[str] = None,
        *,
        service_client: Optional[Client] = None,
        timeout_sec: Optional[float] = None,
    ):
        """Wait for a service to be available.

        Args:
            srv_type: The type of the service.
            srv_name: The name of the service.
            service_client: The service client to use.
            timeout_sec: The timeout in seconds.

        Raises:
            ServiceCallError: If the service call fails.
            ServiceCallUnsuccessfulError: If the service call is unsuccessful.
        """
        # If the service client is not provided, create a new one and destroy
        # it after the service call
        if service_client is None:
            service_client = self._create_client(srv_type, srv_name)
            destroy_service_client = True
        else:
            destroy_service_client = False

        # Wait for the service to be available with the provided or default
        # timeout
        try:
            self.log(
                f"Waiting for {srv_name} service to be available...",
                severity="DEBUG",
            )
            timeout_sec = (
                timeout_sec
                if timeout_sec is not None
                else self.get_parameter_wrapper("default_service_wait_timeout")
            )
            if not service_client.wait_for_service(timeout_sec=timeout_sec):
                error_msg = f"{srv_name} not available!"
                self.log(error_msg, severity="ERROR")
                raise TimeoutError(error_msg)

            self.log(f"{srv_name} service is available", severity="DEBUG")
        finally:
            # Destroy the service client if it was created by this function
            if destroy_service_client:
                self.destroy_client(service_client)

    def service_call(
        self,
        srv_request: SrvTypeRequest,
        srv_type: Optional[type[SrvType]] = None,
        srv_name: Optional[str] = None,
        *,
        service_client: Optional[Client] = None,
        timeout_sec: Optional[float] = None,
    ) -> SrvTypeResponse:
        """Call a service synchronously, returning the response.

        Args:
            srv_request: The request message for the service.
            srv_type: The type of the service.
            srv_name: The name of the service.
            service_client: The service client to use.
            timeout_sec: The timeout in seconds.

        Returns:
            The response from the service.

        Raises:
            ServiceCallError: If the service call fails.
            ServiceCallUnsuccessfulError: If the service call is unsuccessful.
        """
        # If the service client is not provided, create a new one and destroy
        # it after the service call
        if service_client is None:
            service_client = self._create_client(srv_type, srv_name)
            destroy_service_client = True
        else:
            destroy_service_client = False

        # Call the service with the provided or default timeout and
        # validate the response
        try:
            self.log(
                f"Calling {service_client.service_name} service...",
                severity="DEBUG",
            )
            timeout_sec = (
                timeout_sec
                if timeout_sec is not None
                else self.get_parameter_wrapper("default_service_call_timeout")
            )
            response: SrvTypeResponse = service_client.call(
                srv_request, timeout_sec=timeout_sec
            )  # type: ignore
            validate_service_response(response, service_client)
            return response
        finally:
            # Destroy the service client if it was created by this function
            if destroy_service_client:
                self.destroy_client(service_client)

    async def service_call_async(
        self,
        srv_request: SrvTypeRequest,
        srv_type: Optional[type[SrvType]] = None,
        srv_name: Optional[str] = None,
        *,
        service_client: Optional[Client] = None,
        timeout_sec: Optional[float] = None,
    ) -> SrvTypeResponse:
        """Call a service asynchronously, returning a future and the service client.

        Args:
            srv_request: The request message for the service.
            srv_type: The type of the service.
            srv_name: The name of the service.
            service_client: The service client to use.
            timeout_sec: The timeout in seconds.

        Returns:
            The response from the service.

        Raises:
            ServiceCallError: If the service call fails.
            ServiceCallUnsuccessfulError: If the service call is unsuccessful.
        """
        # If the service client is not provided, create a new one and
        # destroy it after the service call
        if service_client is None:
            service_client = self._create_client(srv_type, srv_name)
            destroy_service_client = True
        else:
            if srv_type is not None or srv_name is not None:
                raise ValueError(
                    "srv_type and srv_name must be None if service_client is provided"
                )
            destroy_service_client = False

        # Call the service asynchronously
        self.log(
            f"Calling {service_client.service_name} service asynchronously...",
            severity="DEBUG",
        )
        future = service_client.call_async(srv_request)

        try:
            # Wait asynchronously for the service call to finish with the
            # provided or default timeout
            async with asyncio.timeout(
                timeout_sec
                if timeout_sec is not None
                else self.get_parameter_wrapper("default_service_call_timeout")
            ):
                response: SrvTypeResponse = await future  # type: ignore
            self.log(
                f"Service call to {service_client.service_name} finished with response:",
                severity="DEBUG",
            )
            self.log_ros_msg(
                response,
                title=f"{service_client.service_name} response",
                severity="DEBUG",
            )
            validate_service_response(response, service_client)
            return response
        # except asyncio.CancelledError as e:
        #     self.log(
        #         f"Service call to {srv_name} was cancelled by asyncio",
        #         severity="DEBUG",
        #     )
        #     raise e
        # except TimeoutError as e:
        #     self.log(
        #         f"Service call to {srv_name} timed out",
        #         severity="ERROR",
        #     )
        #     raise e
        finally:
            if destroy_service_client:
                self.destroy_client(service_client)
