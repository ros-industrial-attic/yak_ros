# Number of column pixels in input images
int32 cols
# brief Number of row pixels in input images
int32 rows

# Camera intrinsic parameters [fx, fy, cx, xy]
float32[] intr

# Dimensions of the TSDF volume in voxels. Multiply by resolution to get meters. Length should be 3.
int32[] volume_dims
# Resolution of the TSDF volume voxels (meters/voxel)
float32 volume_resolution

# Static tranform applied to the volume. Not currently supported.
geometry_msgs/Pose volume_pose

# Pixels
float32 bilateral_sigma_depth
# Pixels
float32 bilateral_sigma_spatial
# Pixels
int32 bilateral_kernel_size

# Meters
float32 icp_truncate_depth_dist
# Meters
float32 icp_dist_thres
# Radians
float32 icp_angle_thres
# Iterations for level index 0,1,..,3
int32[] icp_iter_num

# Meters - Camera must move by this amount in order for a new image to be incorporated
float32 tsdf_min_camera_movement
# Meters - TSDF truncation distance
float32 tsdf_trunc_dist
# Frames
int32 tsdf_max_weight

# In voxel sizes
float32 raycast_step_factor
# In voxel sizes
float32 gradient_delta_factor

# Meters - Should be length 3
float32[] light_pose

# If true use the pose given from an external source (e.g. robot kinematics)
bool use_pose_hints
# If true use ICP to correlate new images to the existing TSDF
bool use_icp
# If true only apply new sensor readings when the pose has sufficiently changed from the previous reading
bool update_via_sensor_motion

