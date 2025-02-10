from collections.abc import Awaitable, Callable, Coroutine
from typing import Any, Optional

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from rclpy.exceptions import (
    ParameterAlreadyDeclaredException,
    ParameterNotDeclaredException,
)
from rclpy.node import Node
from rclpy.task import Task as RclpyTask

from tabletop_server.utils import (
    SrvType,
    SrvTypeRequest,
    SrvTypeResponse,
    validate_service_response,
)


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

    def __init__(
        self,
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self._check_parameters()
        self._declare_default_parameters()
        self.log_params()

    def log(
        self,
        message: str,
        severity: str = "INFO",
    ):
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
        for param in params.values():
            self.log(f"{param.name}: {param.value}", severity=severity)  # type: ignore

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
            value = param.value  # type: ignore
            name_parts = name.split(".")
            current_level = nested_params

            for part in name_parts[:-1]:
                current_level = current_level.get(part, {})

            current_level[name_parts[-1]] = value

        return nested_params

    def create_rclpy_task(
        self,
        handle: Callable | Coroutine,
        *args,
        done_callback: Optional[Callable] = None,
        **kwargs,
    ) -> RclpyTask:
        """
        Create a future that will be resolved when the task is finished.
        To wait for the task to finish, await the future.
        """
        rclpy_task = self.executor.create_task(handle, *args, **kwargs)  # type: ignore

        if done_callback is not None:
            rclpy_task.add_done_callback(done_callback)

        return rclpy_task

    def wait_for_service(
        self,
        srv_type: Optional[type] = None,
        srv_name: Optional[str] = None,
        service_client: Optional[Client] = None,
        destroy_service_client: bool = True,
        wait_timeout: Optional[float] = None,
    ):
        """
        Wait for a service to be available.
        """
        service_client = self._create_client(
            srv_type, srv_name, service_client, destroy_service_client
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
            if destroy_service_client:
                self.destroy_client(service_client)

    def _create_client(
        self,
        srv_type: Optional[type] = None,
        srv_name: Optional[str] = None,
        service_client: Optional[Client] = None,
        destroy_service_client: bool = True,
    ) -> Client:
        """
        Create a client for a service or return the provided client if it
        is not None.
        """
        if service_client is None:
            if srv_type is None or srv_name is None:
                self.log(
                    "srv_type and srv_name must be provided if service_client is not provided",
                    severity="ERROR",
                )
                raise ValueError(
                    "srv_type and srv_name must be provided if service_client is not provided"
                )
            service_client = self.create_client(
                srv_type,
                srv_name,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
        else:
            if srv_type is not None or srv_name is not None:
                self.log(
                    "srv_type and srv_name must not be provided if service_client is provided",
                    severity="ERROR",
                )
                raise ValueError(
                    "srv_type and srv_name must not be provided if service_client is provided"
                )
            if destroy_service_client:
                self.log(
                    "destroy_service_client must not be provided if service_client is provided",
                    severity="ERROR",
                )
                raise ValueError(
                    "destroy_service_client must not be provided if service_client is provided"
                )
        return service_client

    def service_call(
        self,
        srv_request: SrvTypeRequest,
        srv_type: Optional[type[SrvType]] = None,
        srv_name: Optional[str] = None,
        service_client: Optional[Client] = None,
        destroy_service_client: bool = True,
        call_timeout: Optional[float] = None,
    ) -> SrvTypeResponse | tuple[SrvTypeResponse, Client]:
        """
        Call a service synchronously, returning the response and optionally
        the service client.
        """
        service_client = self._create_client(
            srv_type, srv_name, service_client, destroy_service_client
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
            response = validate_service_response(response, service_client)
            if destroy_service_client:
                return response
            else:
                return response, service_client
        finally:
            if destroy_service_client:
                self.destroy_client(service_client)

    def service_call_async(
        self,
        srv_request: SrvTypeRequest,
        srv_type: Optional[type[SrvType]] = None,
        srv_name: Optional[str] = None,
        service_client: Optional[Client] = None,
        destroy_service_client: bool = True,
    ) -> Awaitable:
        """
        Call a service asynchronously, returning a future and the service
        client.
        """
        service_client = self._create_client(
            srv_type, srv_name, service_client, destroy_service_client=False
        )
        future = service_client.call_async(srv_request)
        future.add_done_callback(
            lambda _: self.destroy_client(service_client)
            if destroy_service_client
            else None
        )
        return future
