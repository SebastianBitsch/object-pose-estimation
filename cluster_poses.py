import numpy as np
from math import acos, pi

def rotationGeodesicDistance(R1, R2):
    """
    Calculate the geodesic distance between two rotation matrices.
    
    Args:
        R1 (np.ndarray): 3x3 rotation matrix.
        R2 (np.ndarray): 3x3 rotation matrix.
    
    Returns:
        float: Geodesic distance in radians.
    """
    cos = ((R1 @ R2.T).trace() - 1) / 2.0
    cos = max(min(cos, 1.0), -1.0)
    return acos(cos)

def cluster_poses(angle_diff, dist_diff, poses_in, symmetry_tfs):
    """
    Cluster a set of poses based on rotation and translation differences.
    
    Args:
        angle_diff (float): Maximum rotation difference in degrees.
        dist_diff (float): Maximum translation difference in meters.
        poses_in (np.ndarray): List of 4x4 transformation matrices.
        symmetry_tfs (np.ndarray): List of 4x4 symmetry transformation matrices.
    
    Returns:
        np.ndarray: List of clustered poses.
    """
    print(f"num original candidates = {len(poses_in)}")
    poses_out = [poses_in[0]]
    radian_thres = angle_diff / 180.0 * pi

    for i in range(1, len(poses_in)):
        is_new = True
        cur_pose = poses_in[i]
        for cluster in poses_out:
            t0 = cluster[:3, 3]
            t1 = cur_pose[:3, 3]
            if np.linalg.norm(t0 - t1) >= dist_diff:
                continue
            for tf in symmetry_tfs:
                cur_pose_tmp = cur_pose @ tf
                rot_diff = rotationGeodesicDistance(cur_pose_tmp[:3, :3], cluster[:3, :3])
                if rot_diff < radian_thres:
                    is_new = False
                    break
            if not is_new:
                break
        if is_new:
            poses_out.append(poses_in[i])

    print(f"num of pose after clustering: {len(poses_out)}")
    return np.array(poses_out)