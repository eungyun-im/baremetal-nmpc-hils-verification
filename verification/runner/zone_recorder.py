#!/usr/bin/python3
# 삼거리~M3 구간에서만 라이다, 자세, 인지 목록을 저장한다. 마스터가 내려가면 저장하고 끝낸다.
import rospy, rosgraph, numpy as np, json, math, os, sys, time
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
OUT = sys.argv[1]; pose = [None]; scans = []; infos = []
def in_zone(p): return 302500 < p[0] < 302570 and 4123985 < p[1] < 4124055
def on_pose(m):
    q = m.pose.pose.orientation
    pose[0] = (m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
def on_cloud(m):
    p = pose[0]
    if p is None or not in_zone(p): return
    of = {f.name: f.offset for f in m.fields}; a = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
    xyz = np.stack([np.frombuffer(a[:, of[k]:of[k] + 4].tobytes(), dtype="<f4") for k in "xyz"], 1)
    xyz = xyz[np.isfinite(xyz).all(1) & (np.hypot(xyz[:, 0], xyz[:, 1]) < 45)]
    scans.append((rospy.get_time(), p, xyz.astype("f4")))
def on_info(m):
    p = pose[0]
    if p is not None and in_zone(p): infos.append((rospy.get_time(), p, m.data))
rospy.init_node("zone_recorder", anonymous=True, disable_signals=True)
rospy.Subscriber("/ekf_pose_with_covariance", PoseWithCovarianceStamped, on_pose, queue_size=5)
rospy.Subscriber("/velodyne_points", PointCloud2, on_cloud, queue_size=1, buff_size=2**24)
rospy.Subscriber("/obstacle_info", String, on_info, queue_size=5)
while rosgraph.is_master_online(): time.sleep(1)
if scans:
    f = os.path.join(OUT, "zone_%d.npz" % time.time())
    np.savez_compressed(f, t=np.array([s[0] for s in scans]), pose=np.array([s[1] for s in scans]), pts=np.array([s[2] for s in scans], dtype=object), info=json.dumps([[i[0], i[1], i[2]] for i in infos]))
    print("saved", f, len(scans), flush=True)
os._exit(0)
