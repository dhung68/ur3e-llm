"""Calibrated RGB-D color measurements. No simulator object coordinates."""
import math
import cv2
import numpy as np

COLORS = {'red_cube': [(0, 6), (174, 179)], 'yellow_cube': [(20, 38)],
          'blue_cube': [(100, 132)], 'green_cube': [(40, 85)], 'purple_cube': [(135, 173)]}


def camera_rotation(rpy):
    r, p, y = rpy
    rx = np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    ry = np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    rz = np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
    return rz @ ry @ rx


def detect(rgb, depth, intrinsic, extrinsic):
    if rgb.shape[:2] != depth.shape or len(intrinsic) != 9:
        raise ValueError('RGB/depth dimensions or camera_info invalid')
    fx, fy, cx, cy = intrinsic[0], intrinsic[4], intrinsic[2], intrinsic[5]
    if min(fx, fy) <= 0:
        raise ValueError('Uncalibrated camera')
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    rotation = camera_rotation(extrinsic['rpy'])
    origin = np.array(extrinsic['position'])
    objects, errors = {}, {}
    for name, ranges in COLORS.items():
        mask = np.zeros(depth.shape, np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, (lo, 105, 45), (hi, 255, 255))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        components = [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= 35]
        if len(components) != 1:
            errors[name] = 'missing/occluded' if not components else 'ambiguous color components'
            continue
        v, u = np.where(labels == components[0])
        z = depth[v, u]
        valid = np.isfinite(z) & (z > .1) & (z < 3)
        u, v, z = u[valid], v[valid], z[valid]
        if len(z) < 35:
            errors[name] = 'insufficient valid depth'
            continue
        points = np.stack([z, -(u+.5-cx)*z/fx, -(v+.5-cy)*z/fy], axis=1) @ rotation.T + origin
        lo, hi = np.percentile(points, [1, 99], axis=0)
        # Known 30 mm axis-aligned cube; visible front is negative world Y.
        # RGB-D supplies height even when carried; no table-plane substitution.
        center = [(lo[0]+hi[0])/2, lo[1]+.015 if origin[1] < 0 else hi[1]-.015, hi[2]-.015]
        span = hi-lo
        if not (.015 < span[0] < .045 and max(span) < .06):
            errors[name] = 'partial view/invalid cube extent'
            continue
        objects[name] = {'position': center, 'quaternion': [0.,0.,0.,1.],
                         'quality': min(1., len(z)/150, span[0]/.027, span[2]/.027), 'pixels': len(z), 'extent': span.tolist()}
    return objects, errors
