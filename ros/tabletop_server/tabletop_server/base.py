from typing import Any, Optional

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from rclpy.exceptions import (
    ParameterAlreadyDeclaredException,
    ParameterNotDeclaredException,
)
from rclpy.node import Node
from tf2_ros import Future


class BaseNode(Node):
    """
    Base class for all nodes.

    This class extends the Node class with common functionality, including
    parameter declaration, logging, and service calls.
    """

    default_params: dict[str, Any] = {
        "default_service_wait_timeout": 5.0,
        "default_service_call_timeout": 2.0,
    }
    required_params: set[str] = set()

    def __init__(self, *args, executor=None, **kwargs):
        super().__init__(*args, **kwargs)
        self._check_parameters()
        self._declare_default_parameters()
        self.log_params()

        self._executor = executor

    def log(self, message, severity="INFO"):
        """
        Log a message with the given severity.
        """
        if severity == "DEBUG":
            self.get_logger().debug(message)
        elif severity == "INFO":
            self.get_logger().info(message)
        elif severity == "WARN":
            self.get_logger().warning(message)
        elif severity == "ERROR":
            self.get_logger().error(message)
        elif severity == "FATAL":
            self.get_logger().fatal(message)

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
                self.get_parameter(name)
            except ParameterNotDeclaredException:
                self.log(
                    f"Required parameter {name} not declared", severity="ERROR"
                )
                raise RuntimeError(f"Required parameter {name} not declared")

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

    def log_params(self, prefix: str = "", severity: str = "INFO"):
        """
        Log all parameters with the given prefix.
        """
        prefix = (
            f"{prefix}." if prefix and not prefix.endswith(".") else prefix
        )
        params = self.get_parameters_by_prefix(prefix)
        for name, param in params.items():
            self.log(f"{prefix}{name}: {param.value}", severity=severity)

    def get_nested_parameters(self, prefix: str = "") -> dict:
        """
        Retrieves all parameters from a ROS2 node and structures them into a nested dictionary.
        Namespaces are represented as nested dictionaries.
        """
        prefix = (
            f"{prefix}." if prefix and not prefix.endswith(".") else prefix
        )
        params = self.get_parameters_by_prefix(prefix)
        nested_params = {}

        for name, param in params.items():
            value = param.value
            name_parts = name.split(".")
            current_level = nested_params

            for part in name_parts[:-1]:
                current_level = current_level.get(part, {})

            current_level[name_parts[-1]] = value

        return nested_params

    def create_future(self, function, *args, callback=None, **kwargs):
        """
        Create a future that will be resolved when the task is finished.
        To wait for the
        """
        if self._executor is None:
            raise ValueError("Executor not set")

        future = self._executor.create_task(function, *args, **kwargs)
        if callback is not None:
            future.add_done_callback(callback)

        return future

    def wait_for_service(
        self,
        srv_type,
        srv_name,
        wait_timeout=None,
    ):
        """
        Wait for a service to be available.
        """
        service_client = self.create_client(
            srv_type, srv_name, callback_group=MutuallyExclusiveCallbackGroup()
        )
        try:
            # Wait for the service to be available
            self.log(f"Waiting for {srv_name} service to be available...")
            wait_timeout = (
                wait_timeout
                if wait_timeout is not None
                else self.get_parameter("default_service_wait_timeout").value
            )
            if not service_client.wait_for_service(timeout_sec=wait_timeout):
                error_msg = f"{srv_name} not available!"
                self.log(error_msg, severity="ERROR")
                raise TimeoutError(error_msg)

            self.log(f"{srv_name} service is available")
        finally:
            service_client.destroy()

    def service_call(
        self,
        srv_request: Any,
        srv_type: type,
        srv_name: Optional[str] = None,
        service_client: Optional[Client] = None,
        call_timeout: Optional[float] = None,
    ):
        """
        Call a service.
        """
        service_client = (
            self.create_client(
                srv_type,
                srv_name,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
            if service_client is None
            else service_client
        )
        try:
            # Call the service
            self.log(f"Calling {service_client.service_name} service...")
            call_timeout = (
                call_timeout
                if call_timeout is not None
                else self.get_parameter("default_service_call_timeout").value
            )
            response = service_client.call(
                srv_request, timeout_sec=call_timeout
            )

            # Check if the service call succeeded
            if response is None:
                error_msg = (
                    f"{service_client.service_name} service call timed out!"
                )
                self.log(error_msg, severity="ERROR")
                raise TimeoutError(error_msg)
            elif not response.success:
                error_msg = (
                    f"{service_client.service_name} service call failed!"
                )
                self.log(error_msg, severity="ERROR")
                raise RuntimeError(error_msg)
            else:
                self.log(
                    f"{service_client.service_name} service call succeeded"
                )
                return response
        finally:
            service_client.destroy()

    def service_call_async(
        self,
        srv_request: Any,
        srv_type: type,
        srv_name: Optional[str] = None,
        service_client: Optional[Client] = None,
    ):
        """
        Call a service.
        """
        service_client = (
            self.create_client(
                srv_type,
                srv_name,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
            if service_client is None
            else service_client
        )
        future = service_client.call_async(srv_request)
        return future, service_client

    async def wait_for_service_future(
        self,
        future: Future,
        service_client_to_destroy: Client,
    ):
        """
        Wait for a service to be available.
        """
        # Check if the service call succeeded
        try:
            response = await future
            if response is None:
                error_msg = f"{service_client_to_destroy.service_name} service call timed out!"
                self.log(error_msg, severity="ERROR")
                raise TimeoutError(error_msg)
            elif not response.success:
                error_msg = f"{service_client_to_destroy.service_name} service call failed!"
                self.log(error_msg, severity="ERROR")
                raise RuntimeError(error_msg)
            else:
                self.log(
                    f"{service_client_to_destroy.service_name} service call succeeded"
                )
                return response
        finally:
            service_client_to_destroy.destroy()
