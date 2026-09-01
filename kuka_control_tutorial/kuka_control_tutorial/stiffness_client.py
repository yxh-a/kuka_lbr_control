"""Push and read the Cartesian impedance controller's stiffness parameters.

The controller declares its stiffness as six doubles -- ``stiffness.trans_x``
through ``stiffness.rot_z`` -- and, since the runtime-stiffness change, accepts
edits to them while it is active. This wraps the parameter service in the same
blocking style as :class:`MotionClient`, so a worker thread can set a value and
find out whether the controller accepted it.

The controller validates: out-of-range or non-finite values are refused and the
parameter keeps its old value, so a rejection here means nothing changed.
"""

import threading
import time

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters

# Axis order used everywhere: the parameter names and the GUI rows share it.
STIFFNESS_PARAMS = (
    "stiffness.trans_x",
    "stiffness.trans_y",
    "stiffness.trans_z",
    "stiffness.rot_x",
    "stiffness.rot_y",
    "stiffness.rot_z",
)

STIFFNESS_LABELS = ("trans x", "trans y", "trans z", "rot x", "rot y", "rot z")


class StiffnessClient:
    """Blocking accessor for the controller's stiffness parameters.

    Call from a thread that is not spinning the node.
    """

    def __init__(self, node, controller="/lbr/cartesian_impedance_controller"):
        self.node = node
        self.controller = controller.rstrip("/")
        self._set = node.create_client(
            SetParameters, f"{self.controller}/set_parameters"
        )
        self._get = node.create_client(
            GetParameters, f"{self.controller}/get_parameters"
        )

    def wait_for_service(self, timeout_sec=5.0):
        """Wait for both parameter services.

        They live on the same node but are discovered independently, so waiting
        only for one and then immediately using the other loses the race.
        """
        deadline = time.monotonic() + timeout_sec
        if not self._set.wait_for_service(timeout_sec=timeout_sec):
            return False
        remaining = max(0.1, deadline - time.monotonic())
        return self._get.wait_for_service(timeout_sec=remaining)

    def available(self):
        return self._set.service_is_ready() and self._get.service_is_ready()

    def get(self, timeout=3.0):
        """Current stiffness as a list of six floats, or None if unavailable."""
        if not self._get.wait_for_service(timeout_sec=timeout):
            return None
        request = GetParameters.Request(names=list(STIFFNESS_PARAMS))
        response = self._call(self._get, request, timeout)
        if response is None or len(response.values) != len(STIFFNESS_PARAMS):
            return None
        # An undeclared parameter comes back as PARAMETER_NOT_SET.
        if any(v.type != ParameterType.PARAMETER_DOUBLE for v in response.values):
            return None
        return [v.double_value for v in response.values]

    def set(self, values, timeout=3.0):
        """Set all six axes at once.

        Returns ``(True, "")`` or ``(False, reason)``. Sending them in one
        request means the controller validates the batch together, so a bad
        value cannot leave the others half-applied.
        """
        if not self._set.wait_for_service(timeout_sec=timeout):
            return False, f"{self.controller}/set_parameters is not available"

        request = SetParameters.Request(
            parameters=[
                Parameter(
                    name=name,
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)
                    ),
                )
                for name, value in zip(STIFFNESS_PARAMS, values)
            ]
        )
        response = self._call(self._set, request, timeout)
        if response is None:
            return False, "the controller did not answer in time"

        for name, result in zip(STIFFNESS_PARAMS, response.results):
            if not result.successful:
                return False, f"{name} rejected: {result.reason or 'no reason given'}"
        return True, ""

    @staticmethod
    def _call(client, request, timeout):
        """Call a service and block for the reply without spinning the node."""
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            return None
        return future.result()
