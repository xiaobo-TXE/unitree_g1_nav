"""Read and decode the Unitree wireless-controller state."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from unitree_go.msg import WirelessController


BUTTON_BITS = {
    'R1': 0,
    'L1': 1,
    'START': 2,
    'SELECT': 3,
    'R2': 4,
    'L2': 5,
    'F1': 6,
    'F3': 7,
    'A': 8,
    'B': 9,
    'X': 10,
    'Y': 11,
    'UP': 12,
    'RIGHT': 13,
    'DOWN': 14,
    'LEFT': 15,
}


def decode_buttons(keys):
    """Return the names of all buttons set in a uint16 key mask."""
    key_mask = int(keys) & 0xFFFF
    return tuple(
        name for name, bit in BUTTON_BITS.items()
        if key_mask & (1 << bit)
    )


class WirelessControllerReader(Node):
    """Subscribe to joystick axes and emit button press/release edges."""

    def __init__(self):
        super().__init__('g1_wireless_controller')

        self.declare_parameter('wireless_controller_topic', '/wirelesscontroller')
        self.declare_parameter('motion_enable_topic', '/motion_enabled')
        self.declare_parameter('state_log_rate', 5.0)

        controller_topic = self.get_parameter('wireless_controller_topic').value
        motion_enable_topic = self.get_parameter('motion_enable_topic').value
        state_log_rate = float(self.get_parameter('state_log_rate').value)
        if state_log_rate < 0.0:
            raise ValueError('state_log_rate must not be negative')

        self._state_log_period = (1.0 / state_log_rate if state_log_rate > 0.0 else None)
        self._last_state_log_time = None
        self._last_keys = None
        self.latest_state = None

        motion_enable_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._motion_enable_publisher = self.create_publisher(
            Bool, motion_enable_topic, motion_enable_qos)

        self._wireless_controller_subscription = self.create_subscription(
            WirelessController,
            controller_topic,
            self._on_wireless_controller,
            10,
        )

        self.get_logger().info(
            'Reading Unitree wireless controller state from %s'
            % controller_topic)
        self._publish_motion_enabled(False)

    def _on_wireless_controller(self, message):
        self.latest_state = message
        keys = int(message.keys) & 0xFFFF

        # Establish a baseline first, so a button held during startup does not
        # become a false rising edge and trigger a future robot action.
        if self._last_keys is not None:
            pressed_mask = keys & ~self._last_keys & 0xFFFF
            released_mask = self._last_keys & ~keys & 0xFFFF

            for button in decode_buttons(pressed_mask):
                self.get_logger().info('Button pressed: %s' % button)
                self.on_button_pressed(button, message)

            for button in decode_buttons(released_mask):
                self.get_logger().info('Button released: %s' % button)
                self.on_button_released(button, message)

        self._last_keys = keys
        self._log_state_if_due(message, keys)

    def _log_state_if_due(self, message, keys):
        if self._state_log_period is None:
            return

        now = time.monotonic()
        if (self._last_state_log_time is not None and
                now - self._last_state_log_time < self._state_log_period):
            return

        buttons = decode_buttons(keys)
        button_text = ','.join(buttons) if buttons else 'none'
        self.get_logger().info(
            'lx=%.3f ly=%.3f rx=%.3f ry=%.3f '
            'keys=0x%04X buttons=[%s]'
            % (message.lx, message.ly, message.rx, message.ry,
               keys, button_text))
        self._last_state_log_time = now

    def on_button_pressed(self, button, message):
        """Enable motion with START and disable it with B."""
        del message
        if button == 'B':
            self._publish_motion_enabled(False)
            self.get_logger().warning('B pressed: stop requested')
        elif button == 'START':
            self._publish_motion_enabled(True)
            self.get_logger().info('START pressed: motion enabled')

    def _publish_motion_enabled(self, enabled):
        message = Bool()
        message.data = enabled
        self._motion_enable_publisher.publish(message)

    def on_button_released(self, button, message):
        """Handle one release; extend this when release actions are needed."""
        del button, message


def main(args=None):
    rclpy.init(args=args)
    node = WirelessControllerReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
