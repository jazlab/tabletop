import asyncio
from collections.abc import Awaitable, Callable
from inspect import isawaitable, iscoroutinefunction
from typing import Any, Optional

import yaml
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from rclpy.exceptions import (
    ParameterAlreadyDeclaredException,
    ParameterNotDeclaredException,
)
from rclpy.node import Node
from tabletop_utils.ros import (
    SrvType,
    SrvTypeRequest,
    SrvTypeResponse,
    msg_to_dict,
    validate_service_response,
)

DEFAULT_LOG_SEVERITY = "INFO"


class BracketedListDumper(yaml.Dumper):
    """
    Custom YAML Dumper that formats scalar sequences as bracketed lists.
    """

    def represent_sequence(self, tag, sequence, flow_style=None):
        """
        Overrides the default represent_sequence to use flow style (bracketed)
        for sequences containing only scalar values.
        """
        if all(
            isinstance(item, (str, int, float, bool, type(None)))
            for item in sequence
        ):
            return yaml.Dumper.represent_sequence(
                self, tag, sequence, flow_style=True
            )
        else:
            return yaml.Dumper.represent_sequence(
                self, tag, sequence, flow_style=flow_style
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
        "yaml_width": 120,
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
        self.log_params(severity="DEBUG")

    def log(
        self,
        message: Any,
        severity: str = DEFAULT_LOG_SEVERITY,
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
                self.get_parameter_wrapper(name)
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

            current_level[keys[-1]] = value

        return nested_params

    def get_parameter_wrapper(self, name: str) -> Any:
        """
        Get a parameter from the node.
        """
        try:
            value = self.get_parameter(name).value
            if value == "null":
                return None
            else:
                return value
        except ParameterNotDeclaredException:
            return self.get_nested_parameters(name)

    def log_ros_msg(
        self,
        msg: Any,
        title: Optional[str] = None,
        severity: str = DEFAULT_LOG_SEVERITY,
    ):
        string = yaml.dump(
            msg_to_dict(msg),
            Dumper=BracketedListDumper,
            width=self.get_parameter_wrapper("yaml_width"),
        )
        if title is not None:
            string = f"{title}\n{string}"
        self.log(string, severity=severity)

    @staticmethod
    def rclpy_task_wrapper(
        handle: Callable,
        *args,
        **kwargs,
    ):
        try:
            return handle(*args, **kwargs)
        except Exception as e:
            print(f"Error in {handle.__name__} during callback:")
            print(e)
            return e

    @staticmethod
    async def rclpy_task_wrapper_awaitable(handle: Awaitable):
        try:
            return await handle
        except Exception as e:
            try:
                print(f"Error in {handle.__name__} during callback:")  # type: ignore
            except AttributeError:
                print(f"Error in {handle} during callback:")
            print(e)
            return e

    async def create_rclpy_task(
        self,
        handle: Callable | Awaitable,
        *args,
        done_callback: Optional[Callable] = None,
        **kwargs,
    ) -> Any:
        """
        Create a coroutine that will be resolved when the task is finished.
        To wait for the task to finish, await the coroutine.
        """
        if isawaitable(handle):
            if args or kwargs:
                raise ValueError(
                    "Arguments and keyword arguments are not allowed for awaitable handles"
                )
            rclpy_task = self.executor.create_task(  # type: ignore
                self.rclpy_task_wrapper_awaitable(handle)
            )
        elif iscoroutinefunction(handle):
            rclpy_task = self.executor.create_task(  # type: ignore
                self.rclpy_task_wrapper_awaitable(handle(*args, **kwargs))
            )
        else:
            rclpy_task = self.executor.create_task(  # type: ignore
                self.rclpy_task_wrapper, handle, *args, **kwargs
            )

        if done_callback is not None:
            rclpy_task.add_done_callback(done_callback)

        result = await rclpy_task
        if isinstance(result, Exception):
            raise result
        else:
            return result

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
                else self.get_parameter_wrapper("default_service_wait_timeout")
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
        timeout_sec: Optional[float] = None,
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
            timeout_sec = (
                timeout_sec
                if timeout_sec is not None
                else self.get_parameter_wrapper("default_service_call_timeout")
            )
            response = service_client.call(
                srv_request, timeout_sec=timeout_sec
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

    async def service_call_async(
        self,
        srv_request: SrvTypeRequest,
        srv_type: Optional[type[SrvType]] = None,
        srv_name: Optional[str] = None,
        service_client: Optional[Client] = None,
        destroy_service_client: bool = True,
    ) -> SrvTypeResponse:
        """
        Call a service asynchronously, returning a future and the service
        client.
        """
        try:
            service_client = self._create_client(
                srv_type, srv_name, service_client, destroy_service_client
            )
            future = service_client.call_async(srv_request)
            future.add_done_callback(
                lambda _: self.destroy_client(service_client)
                if destroy_service_client
                else None
            )
            return await future  # type: ignore
        except asyncio.CancelledError as e:
            future.cancel()
            # Check if the future was successfully cancelled (avoids race
            # condition)
            if future.done():
                self.log(
                    f"Service call to {srv_name} finished before cancellation",
                    severity="WARN",
                )
                return future.result()  # type: ignore
            else:
                self.log(
                    f"Service call to {srv_name} was cancelled by asyncio",
                    severity="ERROR",
                )
                raise e
