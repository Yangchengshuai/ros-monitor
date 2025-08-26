from typing import Callable, Dict, Any

try:
    import rospy  # type: ignore
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs.point_cloud2 as pc2  # type: ignore
except Exception:  # pragma: no cover
    rospy = None

class LidarSubscriber:
    def __init__(self, topic: str, cb: Callable[[Dict[str, Any]], None]):
        self._cb = cb
        if rospy is None:
            return
        self._sub = rospy.Subscriber(topic, PointCloud2, self._on_msg, queue_size=1)

    def _on_msg(self, msg: 'PointCloud2') -> None:  # type: ignore[name-defined]
        # Lightweight sample: take first N points to avoid flooding
        max_points = 5000
        pts = []
        for i, p in enumerate(pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)):
            if i >= max_points:
                break
            pts.append([float(p[0]), float(p[1]), float(p[2])])
        payload = {
            'timestamp': msg.header.stamp.to_sec(),
            'frame_id': msg.header.frame_id,
            'point_count': len(pts),
            'fields': [{'name':'x','offset':0,'datatype':7,'count':1},
                       {'name':'y','offset':4,'datatype':7,'count':1},
                       {'name':'z','offset':8,'datatype':7,'count':1}],
            'data': pts,
            'compression': 'none'
        }
        self._cb(payload)
