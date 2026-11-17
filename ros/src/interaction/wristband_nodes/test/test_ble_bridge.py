"""Bridge regression tests that run without ROS or a Bluetooth adapter."""

import asyncio
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import AsyncMock, Mock, patch


class FakeNode:
    def __init__(self, name):
        self.logger = Mock()

    def create_subscription(self, *args):
        return None

    def get_logger(self):
        return self.logger


def load_bridge():
    dependencies = {name: ModuleType(name) for name in (
        'bleak', 'rclpy', 'rclpy.node', 'std_msgs', 'std_msgs.msg',
    )}
    dependencies['bleak'].BleakClient = Mock()
    dependencies['bleak'].BleakScanner = Mock()
    dependencies['rclpy.node'].Node = FakeNode
    dependencies['std_msgs.msg'].UInt8MultiArray = object
    source = Path(__file__).parents[1] / 'wristband_nodes' / 'ble_bridge.py'
    spec = importlib.util.spec_from_file_location('bridge_under_test', source)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, dependencies):
        spec.loader.exec_module(module)
    return module


bridge = load_bridge()


class BleBridgeTests(unittest.TestCase):
    def setUp(self):
        self.node = bridge.BleBridge()
        self.node.connected = True

    def send(self, *values):
        self.node.send_vibration(SimpleNamespace(data=list(values)))

    def test_encodes_each_direction_and_magnitude_limits(self):
        for side, direction in enumerate('lrfb'):
            for magnitude in (0, 180, 255):
                self.send(side, magnitude)
                self.assertEqual(
                    self.node.pending_frames[side],
                    f'<{direction}:{magnitude}>'.encode('ascii'),
                )

    def test_side_change_preserves_stop_for_previous_side(self):
        self.send(0, 180)
        self.send(0, 0)
        self.send(1, 200)
        self.assertEqual(list(self.node.pending_frames.values()), [b'<l:0>', b'<r:200>'])

    def test_queue_is_bounded_by_motor_count_and_keeps_newest_values(self):
        for magnitude in range(256):
            for side in range(4):
                self.send(side, magnitude)
        self.assertEqual(len(self.node.pending_frames), 4)
        self.assertEqual(list(self.node.pending_frames.values()), [
            b'<l:255>', b'<r:255>', b'<f:255>', b'<b:255>',
        ])

    def test_invalid_lengths_and_sides_are_ignored(self):
        for values in ((), (0,), (0, 1, 2), (4, 180), (255, 180)):
            self.send(*values)
        self.assertFalse(self.node.pending_frames)
        self.assertEqual(self.node.logger.warning.call_count, 5)

    def test_disconnected_commands_are_dropped(self):
        self.node.connected = False
        self.send(0, 180)
        self.assertFalse(self.node.pending_frames)

    def test_ble_writes_both_sides_and_clears_state_after_disconnect(self):
        node = self.node
        writes = []
        characteristic = SimpleNamespace(properties=['write'])

        class Client:
            services = SimpleNamespace(get_characteristic=Mock(return_value=characteristic))

            async def __aenter__(self):
                return self

            async def __aexit__(self, *args):
                pass

            @property
            def is_connected(self):
                if not writes:
                    node.send_vibration(SimpleNamespace(data=[0, 0]))
                    node.send_vibration(SimpleNamespace(data=[1, 200]))
                return len(writes) < 2

            async def write_gatt_char(self, target, frame, response):
                if target is not characteristic or not response:
                    raise AssertionError('Expected write with response to the configured characteristic')
                writes.append(frame)

        with patch.object(bridge.BleakScanner, 'find_device_by_name', AsyncMock(return_value=object())), \
                patch.object(bridge, 'BleakClient', return_value=Client()), \
                patch.object(bridge.rclpy, 'ok', lambda: len(writes) < 2, create=True):
            asyncio.run(node.run())

        self.assertEqual(writes, [b'<l:0>', b'<r:200>'])
        self.assertFalse(node.connected)
        self.assertFalse(node.pending_frames)
        Client.services.get_characteristic.assert_called_once_with(bridge.WRITE_CHARACTERISTIC_UUID)


if __name__ == '__main__':
    unittest.main()
