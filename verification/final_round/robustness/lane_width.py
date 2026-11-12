#!/usr/bin/env python3
"""K-City 지도(MGeo lane_boundary_set)로 주행 궤적의 차로 폭 W 를 잰다. 290 ms 기준의 가정값 W=3.5 m 확인용.
  python3 lane_width.py <bag>   -> lane_width_<bag>.csv, stdout 요약"""
import json, sys, os
import numpy as np
import rosbag
O = np.array([302595.0, 4124145.0])
J = json.load(open(os.path.expanduser('~/catkin_ws/src/kcity_localizer/data/lane_boundary_set.json')))
segs = []
for b in J:
    p = np.array(b['points'])[:, :2]
    if len(p) >= 2: segs.append(np.hstack([p[:-1], p[1:]]))
S = np.vstack(segs); A, B = S[:, :2], S[:, 2:]
mid = (A + B) / 2
def side_dist(pt, hd):
    near = np.where(np.hypot(*(mid - pt).T) < 30)[0]
    a, b = A[near], B[near]; ab = b - a; L2 = (ab ** 2).sum(1); L2[L2 == 0] = 1e-9
    t = np.clip(((pt - a) * ab).sum(1) / L2, 0, 1); q = a + ab * t[:, None]
    d = pt - q; dist = np.hypot(*d.T)
    n = np.array([-np.sin(hd), np.cos(hd)])        # 왼쪽 법선
    s = (q - pt) @ n
    # 진행 방향과 거의 수직인 투영점(옆쪽)만
    along = np.abs((q - pt) @ np.array([np.cos(hd), np.sin(hd)]))
    ok = along < 0.5
    left = dist[ok & (s > 0.05)]; right = dist[ok & (s < -0.05)]
    return (left.min() if len(left) else np.nan), (right.min() if len(right) else np.nan)
bag = sys.argv[1]
xs, ts, vs, wz = [], [], [], []
with rosbag.Bag(bag) as bg:
    for tp, m, st in bg.read_messages(topics=['/ekf_pose_with_covariance', '/Competition_topic', '/imu']):
        if tp == '/ekf_pose_with_covariance' and m.pose.pose.position.x > 1000:
            q = m.pose.pose.orientation; yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            xs.append((st.to_sec(), m.pose.pose.position.x - O[0], m.pose.pose.position.y - O[1], yaw))
        elif tp == '/Competition_topic': vs.append((st.to_sec(), m.velocity.x))
        elif tp == '/imu': wz.append((st.to_sec(), m.angular_velocity.z))
X = np.array(xs)[::5]; V = np.array(vs); Wz = np.array(wz)
v = np.interp(X[:, 0], V[:, 0], V[:, 1]); r = np.interp(X[:, 0], Wz[:, 0], Wz[:, 1])
rows = []
for (t, x, y, hd), vv, rr in zip(X, v, r):
    l, rt = side_dist(np.array([x, y]), hd)
    R = abs(vv / rr) if abs(rr) > 1e-3 else np.inf
    rows.append((t, x, y, vv, R, l, rt, l + rt))
R_ = np.array(rows)
np.savetxt('lane_width_%s.csv' % os.path.basename(bag)[:-4], R_, delimiter=',', header='t,x,y,v,R,left,right,W', comments='')
W = R_[:, 7]; ok = ~np.isnan(W)
print('points', len(W), 'with both sides', ok.sum())
print('W all: p5 %.2f median %.2f p95 %.2f' % tuple(np.percentile(W[ok], [5, 50, 95])))
tight = ok & (R_[:, 4] < 15) & (R_[:, 3] > 4)
if tight.any(): print('R<15 m & v>4: n %d, W p5 %.2f median %.2f min %.2f, R min %.1f' % (tight.sum(), *np.percentile(W[tight], [5, 50]), W[tight].min(), R_[tight, 4].min()))
