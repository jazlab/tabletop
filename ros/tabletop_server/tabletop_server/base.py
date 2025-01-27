from typing import Any

from rclpy.exceptions import (
    ParameterAlreadyDeclaredException,
    ParameterNotDeclaredException,
)
from rclpy.node import Node


class BaseNode(Node):
    default_params: dict[str, Any] = {
        "default_service_wait_timeout": 5.0,
        "default_service_call_timeout": 2.0,
    }
    required_params: set[str] = set()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._check_parameters()
        self._declare_default_parameters()
        self.log_params()

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

    def service_wait_and_call(
        self,
        srv_type,
        srv_name,
        srv_request,
        wait_timeout=None,
        call_timeout=None,
    ):
        """
        Wait for a service to be available and call it.
        """
        service_client = self.create_client(srv_type, srv_name)
        try:
            # Wait for the service to be available
            self.log(
                f"Waiting for {srv_name} service to be available...",
                severity="INFO",
            )
            wait_timeout = (
                wait_timeout
                if wait_timeout is not None
                else self.get_parameter("default_service_wait_timeout").value
            )
            if not service_client.wait_for_service(timeout_sec=wait_timeout):
                error_msg = f"{srv_name} not available!"
                self.log(error_msg, severity="ERROR")
                raise TimeoutError(error_msg)

            self.log(f"{srv_name} service is available", severity="INFO")

            # Call the service
            self.log(f"Calling {srv_name} service...", severity="INFO")
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
                error_msg = f"{srv_name} service call timed out!"
                self.log(error_msg, severity="ERROR")
                raise TimeoutError(error_msg)
            elif not response.success:
                error_msg = f"{srv_name} service call failed!"
                self.log(error_msg, severity="ERROR")
                raise RuntimeError(error_msg)
            else:
                self.log(f"{srv_name} service call succeeded")
                return response
        finally:
            service_client.destroy()
