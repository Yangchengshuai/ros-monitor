from typing import Callable, Dict, Any

try:
    import rospy  # type: ignore
    from sensor_msgs.msg import Imu
except Exception:  # pragma: no cover
    rospy = None

class ImuSubscriber:
    def __init__(self, topic: str, cb: Callable[[Dict[str, Any]], None]):
        self._cb = cb
        if rospy is None:
            return
        self._sub = rospy.Subscriber(topic, Imu, self._on_msg, queue_size=10)

    def _on_msg(self, msg: 'Imu') -> None:  # type: ignore[name-defined]
        payload = {
            'timestamp': msg.header.stamp.to_sec(),
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w,
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z,
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z,
            },
        }
        self._cb(payload)
