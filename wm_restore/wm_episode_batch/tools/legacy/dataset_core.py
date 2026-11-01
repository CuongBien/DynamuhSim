"""Portable timing and depth geometry; no simulator/ROS dependency."""
import numpy as np


def causal_indices(stamp_ns, available_ns, queries_ns, max_age_ns):
    stamps = np.asarray(stamp_ns, dtype=np.int64)
    available = np.maximum(stamps, np.asarray(available_ns, dtype=np.int64))
    order = np.argsort(available, kind='stable')
    queries = np.asarray(queries_ns, dtype=np.int64)
    j = np.searchsorted(available[order], queries, side='right') - 1
    good = j >= 0
    idx = order[np.maximum(j, 0)]
    good &= queries - stamps[idx] <= max_age_ns
    good &= stamps[idx] <= queries
    good &= available[idx] <= queries
    return idx, good


def interpolate_labels(stamp_ns, values, queries_ns, max_gap_ns):
    times = np.asarray(stamp_ns, dtype=np.int64)
    values = np.asarray(values)
    queries = np.asarray(queries_ns, dtype=np.int64)
    j = np.searchsorted(times, queries, side='right')
    lo = np.clip(j - 1, 0, len(times) - 1)
    hi = np.clip(j, 0, len(times) - 1)
    gap = times[hi] - times[lo]
    valid = (queries >= times[0]) & (queries <= times[-1]) & (gap <= max_gap_ns)
    weight = (queries - times[lo]) / np.maximum(gap, 1)
    weight = weight.reshape(weight.shape + (1,) * (values.ndim - 1))
    return values[lo] * (1 - weight) + values[hi] * weight, valid


def rotation(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def depth_points(depth, k, transform, stride=2):
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride]
    good = np.isfinite(z) & (z >= .28) & (z <= 5.)
    z, u, v = z[good], u[good], v[good]
    optical = np.stack(((u-k[2])*z/k[0], (v-k[5])*z/k[4], z), axis=-1)
    return optical @ transform[:3, :3].T + transform[:3, 3]


def observed_bev(depth, k, transform):
    # 64x64, x forward [0,6.4), y left [-3.2,3.2). Unknown stays unknown.
    grid = np.full((64,64), -1, dtype=np.int8)
    pts = depth_points(depth, k, transform, stride=4)
    inside = (pts[:,0] >= 0) & (pts[:,0] < 6.4) & (pts[:,1] >= -3.2) & (pts[:,1] < 3.2)
    pts = pts[inside]
    ray_pts = pts[(pts[:,2] >= -.05) & (pts[:,2] <= 1.8)]
    start = transform[:2,3]
    # 160 substeps cover the bounded BEV at < .05m. No missing/infinite ray carving.
    rays = start[:,None] + (ray_pts[:,:2,None]-start[:,None]) * (np.arange(160)/160.)
    cells = np.floor((rays.transpose(0,2,1).reshape(-1,2)+[0,3.2])/.1).astype(int)
    valid = (cells[:,0]>=0)&(cells[:,0]<64)&(cells[:,1]>=0)&(cells[:,1]<64)
    cells = cells[valid]
    grid[cells[:,0],cells[:,1]] = 0
    occupied = pts[(pts[:,2]>=.03)&(pts[:,2]<=1.8)]
    cells = np.floor((occupied[:,:2]+[0,3.2])/.1).astype(int)
    grid[cells[:,0],cells[:,1]] = 1
    return grid


def validate_clock(clock_ns):
    times = np.asarray(clock_ns, dtype=np.int64)
    if len(times) < 2 or np.any(np.diff(times) < 0):
        raise ValueError('Clock reset or missing clock: split episode/epoch required')


def group_membership(groups, ids):
    modes = {'none': 0, 'moving': 1, 'walking': 1, 'standing': 2}
    membership = {}
    for group in groups:
        mode = modes[group['type']]
        for human in group['members']:
            if human in membership:
                raise ValueError('Human belongs to multiple simultaneous groups')
            membership[human] = (group['id'], mode)
    return ([membership.get(h,(-1,0))[0] for h in ids],
            [membership.get(h,(-1,0))[1] for h in ids])
