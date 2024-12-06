import open3d as o3d
import numpy as np


def preprocess_point_cloud(pcd, voxel_size: float) -> tuple:
    """
    Preprocess point cloud by downsampling and computing FPFH features
    TODO: Still has a lot of weird hard coded parameters that would be nice to remove

    Args:
        pcd (o3d.geometry.PointCloud): Input point cloud
        voxel_size (float): Voxel size for downsampling
    
    Returns:
        tuple: Downsampled point cloud and its FPFH features
    """
    # Downsample the point cloud
    pcd_down = pcd.voxel_down_sample(voxel_size)
    
    # Estimate normals
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=100)
    )
    if len(pcd.normals) == 0:
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=2 * voxel_size, max_nn=100))

    # Compute FPFH features
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100)
    )
    
    return pcd_down, pcd_fpfh


def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    """
    Perform global registration using RANSAC and Fast Point Feature Histograms
    TODO: Still has a lot of weird hard coded parameters that would be nice to remove

    Args:
        source_down (o3d.geometry.PointCloud): Downsampled source point cloud
        target_down (o3d.geometry.PointCloud): Downsampled target point cloud
        source_fpfh (o3d.pipelines.registration.Feature): Source FPFH features
        target_fpfh (o3d.pipelines.registration.Feature): Target FPFH features
        voxel_size (float): Voxel size used for downsampling
    
    Returns:
        o3d.pipelines.registration.RegistrationResult: Registration result
    """
    distance_threshold = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source = source_down, 
        target = target_down, 
        source_feature = source_fpfh, 
        target_feature = target_fpfh, 
        mutual_filter = True,
        max_correspondence_distance = distance_threshold,
        estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n = 3, # Num random samples to match
        checkers = [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ], 
        criteria = o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)
    )
    if np.isclose(result.transformation, np.eye(4)).all():
        print("Failed to estimate global transformation")

    return result


def refine_registration(source, target, initial_transformation, distance_threshold):
    """
    Refine registration using point-to-point ICP
    
    Args:
        source (o3d.geometry.PointCloud): Source point cloud
        target (o3d.geometry.PointCloud): Target point cloud
        initial_transformation (numpy.ndarray): Initial transformation matrix
        distance_threshold (float): 
    
    Returns:
        o3d.pipelines.registration.RegistrationResult: Refined registration result
    """
    result = o3d.pipelines.registration.registration_icp(
        source = source, 
        target = target, 
        max_correspondence_distance = distance_threshold, 
        init = initial_transformation,
        estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    
    return result


def visualize_registration(source, target, transformation, paint: bool = True) -> None:
    """
    Visualize the registration result
    
    Args:
        source (o3d.geometry.PointCloud): Source point cloud
        target (o3d.geometry.PointCloud): Target point cloud
        transformation (numpy.ndarray): Transformation matrix
    """
    source_temp = o3d.geometry.PointCloud(source)
    target_temp = o3d.geometry.PointCloud(target)

    source_temp.transform(transformation)

    if paint:
        source_temp.paint_uniform_color([1, 0.706, 0])      # yellow
        target_temp.paint_uniform_color([0, 0.651, 0.929])  # blue
    
    o3d.visualization.draw_geometries([source_temp, target_temp])