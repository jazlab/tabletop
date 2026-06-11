"""Base interface class for ROS2 node interfaces.

This module provides the foundational BaseInterface class that all other
interfaces in the tabletop_rig package inherit from. It establishes the
pattern of interfaces being associated with a parent node and having
their own namespaced logger.

The interface pattern separates concerns by grouping related functionality
(e.g., MoveIt operations, hardware communication) into distinct classes
that operate on behalf of a node.
"""

from typing import Any, Optional

from rclpy.exceptions import (
    ParameterNotDeclaredException,
)
from rclpy.impl.rcutils_logger import RcutilsLogger

from tabletop_py.utils.common import dict_update_recursive
from tabletop_rig.nodes.base import BaseNode
from tabletop_rig.utils.logging import LoggerMixin


class BaseInterface(LoggerMixin):
    """Base class for interfaces that operate on behalf of a ROS2 node.

    Interfaces encapsulate specific functionality (motion planning, hardware
    communication, etc.) and operate using a parent node's ROS2 resources
    (publishers, subscribers, services, etc.). Each interface gets its own
    child logger for namespaced log output.

    This class implements LoggerMixin to provide convenient logging methods.

    Attributes:
        _node: The parent ROS2 node this interface operates on behalf of.
        _logger: Child logger for this interface's log output.

    Example:
        class MyInterface(BaseInterface):
            def __init__(self, node: BaseNode):
                super().__init__(node, "my_interface")
                # Create ROS resources using self._node
                self._pub = self._node.create_publisher(...)

            def do_something(self):
                self.log("Doing something", severity="DEBUG")
    """

    def __init__(
        self,
        node: BaseNode,
        name: str,
        *,
        parameter_fallback_prefix: Optional[str] = None,
    ) -> None:
        """Initialize the interface with a parent node.

        Args:
            node: The parent ROS2 node that owns this interface. The interface
                will use the node's resources for ROS communication.
            name: Name of this interface. Used to set the logger name and
                retrieve parameters with the name as prefix (via param()).
            parameter_fallback_prefix: Optional prefix for parameter fallback
                lookup. If a parameter is not found under the interface name,
                the fallback prefix is tried (e.g., 'common_<kind>_interface').
        """
        self._node = node
        self._name = name
        self._parameter_fallback_prefix = parameter_fallback_prefix

        self._logger = node.get_logger().get_child(name)
        self._node.register_interface(self)  # type: ignore

    def get_logger(self) -> RcutilsLogger:
        """Return the logger instance for this interface.

        Required by LoggerMixin to provide logging functionality.

        Returns:
            The RcutilsLogger instance for this interface.
        """
        return self._logger

    @property
    def node(self) -> BaseNode:
        """The parent node this interface operates on behalf of.

        Returns:
            The BaseNode instance that owns this interface.
        """
        return self._node

    def param(self, name: str) -> Any:
        """Get a parameter value from the node, prefixed by the interface name.

        Resolves parameters by trying the interface-specific name first
        (e.g., '<interface_name>.<param_name>'), then falling back to
        the optional fallback prefix if available (e.g.,
        'common_<interface_kind>_interface.<param_name>'). If name is an
        empty string, all parameters for this interface are returned as a dict.

        Args:
            name: Parameter name (may be dot-separated for nested access).

        Returns:
            Parameter value, or nested dict if name is a prefix.
            Returns None for parameters set to "null" string.

        Raises:
            ParameterNotDeclaredException: If the parameter is not found in
                either the interface-specific or fallback namespace.
            ValueError: If the interface-specific and fallback parameters
                have mismatched types (one is a dict, the other is not).
        """
        if name != "":
            name = f".{name}"

        param_name = f"{self._name}{name}"
        fallback_name = f"{self._parameter_fallback_prefix}{name}"

        try:
            param = self._node.param(param_name)
        except ParameterNotDeclaredException:
            if self._parameter_fallback_prefix is not None:
                return self._node.param(fallback_name)
            else:
                raise

        if self._parameter_fallback_prefix is not None:
            try:
                fallback_param = self._node.param(fallback_name)
            except ParameterNotDeclaredException:
                return param
        else:
            return param

        if isinstance(param, dict):
            if not isinstance(fallback_param, dict):
                raise ValueError(
                    f"If parameter {param_name} is a dictionary (i.e. is a "
                    f"parameter prefix for one or more parameters), then "
                    f"fallback parameter {fallback_name} should also be a dictionary"
                )
            return dict_update_recursive(fallback_param, param)
        else:
            if isinstance(fallback_param, dict):
                raise ValueError(
                    f"If parameter {param_name} is a not a dictionary (i.e. "
                    f"it is not a parameter prefix for any parameters), then "
                    f"fallback parameter {fallback_name} should not be a dictionary"
                )
            return param

    def destroy_interface(self):
        """Clean up interface resources.

        Subclasses should override this to destroy ROS clients, subscribers,
        publishers, and other resources. Always call super().destroy_interface()
        at the end to maintain the inheritance chain.
        """
        pass
