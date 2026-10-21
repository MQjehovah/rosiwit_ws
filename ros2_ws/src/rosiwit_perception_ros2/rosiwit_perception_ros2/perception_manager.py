"""Perception manager: unified health/observability for all perception modules.

订阅 ``/perception/heartbeat``（各模块 JSON 心跳），汇总为：

- ``/perception/status``  : std_msgs/String (JSON)
- ``/perception/healthy`` : std_msgs/Bool
- ``/diagnostics``        : diagnostic_msgs/DiagnosticArray
- 服务 ``~/get_status``   : std_srvs/Trigger（message 返回 JSON）

模块本身保持独立进程/独立可执行文件，manager 只做观察与汇总，
不参与数据面，因此某个模块崩溃不会拖垮其他模块。
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class PerceptionManager(Node):
    def __init__(self):
        super().__init__('perception_manager')

        self.declare_parameter('modules', ['avm', 'ground_texture', 'qr_detector'])
        self.declare_parameter('timeout', 3.0)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('heartbeat_topic', '/perception/heartbeat')
        self.declare_parameter('status_topic', '/perception/status')
        self.declare_parameter('healthy_topic', '/perception/healthy')
        self.declare_parameter('diagnostics_topic', '/diagnostics')

        self.expected = list(self.get_parameter('modules').value)
        self.timeout = float(self.get_parameter('timeout').value)
        rate = max(float(self.get_parameter('publish_rate').value), 0.1)

        self._modules = {}
        self._last_healthy = None

        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        self._healthy_pub = self.create_publisher(
            Bool, self.get_parameter('healthy_topic').value, 10)
        self._diag_pub = self.create_publisher(
            DiagnosticArray, self.get_parameter('diagnostics_topic').value, 10)

        self.create_subscription(
            String, self.get_parameter('heartbeat_topic').value,
            self._heartbeat_callback, 50)
        self.create_service(Trigger, '~/get_status', self._get_status_callback)
        self.create_timer(1.0 / rate, self._publish_status)

        self.get_logger().info(
            f'Monitoring modules {self.expected} (timeout={self.timeout:.1f}s)')

    def _heartbeat_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError) as exc:
            self.get_logger().warn(f'Bad heartbeat: {exc}', once=True)
            return
        name = data.get('module')
        if not name:
            return
        self._modules[name] = {
            'last_seen': time.monotonic(),
            'data': data,
        }

    def _status_snapshot(self):
        now = time.monotonic()
        modules = {}
        healthy = True

        for name in self.expected:
            entry = self._modules.get(name)
            if entry is None:
                modules[name] = {
                    'alive': False,
                    'age': None,
                    'state': 'MISSING',
                }
                healthy = False
                continue

            age = now - entry['last_seen']
            alive = age <= self.timeout
            if not alive:
                state = 'STALE'
                healthy = False
            elif age > self.timeout * 0.7:
                state = 'WARN'
            else:
                state = 'OK'

            info = dict(entry['data'])
            info.pop('module', None)
            info.update({'alive': alive, 'age': round(age, 2), 'state': state})
            modules[name] = info

        return healthy, modules

    def _publish_status(self):
        healthy, modules = self._status_snapshot()
        payload = {
            'stamp': round(time.monotonic(), 3),
            'healthy': healthy,
            'modules': modules,
        }
        self._status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))
        self._healthy_pub.publish(Bool(data=healthy))
        self._publish_diagnostics(modules)

        if self._last_healthy is not healthy:
            if healthy:
                self.get_logger().info('All perception modules healthy')
            else:
                bad = [n for n, m in modules.items() if m['state'] != 'OK']
                self.get_logger().warn(f'Perception degraded: {bad}')
            self._last_healthy = healthy

    def _publish_diagnostics(self, modules):
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()

        for name, info in modules.items():
            status = DiagnosticStatus()
            status.name = f'rosiwit_perception/{name}'
            status.hardware_id = name
            state = info['state']
            status.level = {
                'OK': DiagnosticStatus.OK,
                'WARN': DiagnosticStatus.WARN,
                'STALE': DiagnosticStatus.ERROR,
                'MISSING': DiagnosticStatus.ERROR,
            }.get(state, DiagnosticStatus.WARN)
            status.message = state
            for key, value in info.items():
                if key in ('state',):
                    continue
                status.values.append(KeyValue(
                    key=key,
                    value='null' if value is None else str(value)))
            diag.status.append(status)

        self._diag_pub.publish(diag)

    def _get_status_callback(self, request, response):
        healthy, modules = self._status_snapshot()
        response.success = healthy
        response.message = json.dumps(
            {'healthy': healthy, 'modules': modules}, ensure_ascii=False)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
