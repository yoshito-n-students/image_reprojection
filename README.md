# image-reprojection
A ROS nodelet to reproject image as if viewing from different viewpoint or with different camera

## Reprojection Algorithm
1. Calculate mapping between source and destination image pixels
    1. Project destination image pixels to 3D rays by using a **destination camera model**
    1. Calculate intersections between 3D rays from the destination camera and a surface by using the **surface model**
    1. Exclude intersections which are not visible from a source camera by using the same **surface model**
    1. Project intersection points visible from the source camera to source image pixels by using the **source camera model**
1. Generate a destination image by mapping source to destination image pixels

* Models used in 1-i to 1-iv are implemented as plugins so user can use own models as well as standard models described below
* 1 and 2 can be performed either sequentially or simultaneously

## Nodelet: ImageReprojection

### Subsribed topics
**src_imageN** <N = 0, 1, ..> (sensor_msgs/Image)
* subscribed if the parameter ~src_cameraN/model is defined
* sensor_msgs/CameraInfo will also be subscribed by using image_transport::CameraSubscriber

**surface** (arbitrary type)
* type must be supported by the surface model specified by the parameter ~surface/model

### Published topics
**dst_image** (sensor_msgs/Image)
* sensor_msgs/CameraInfo will also be published by using image_transport::CameraPublisher

### Services
**set_dst_camera_info** (sensor_msgs/SetCameraInfo)

### Parameters
**~src_cameraN/model** <N = 1, 2, ..> (string, required for N = 0)
* name or type of the source camera model plugin whose base type is image_reprojection::CameraModel
* other parameters for the source camera model can be defined in the same namespace

**~src_cameraN/image_transport** <N = 1, 2, ..> (string, default: "raw")
* transport type for images of the Nth source camera

**~surface/model** (string, required)
* name or type of the surface model plugin whose base type is image_reprojection::SurfaceModel
* other parameters for the surface model can be defined in the same namespace

**~dst_camera/model** (string, required)
* name or type of the destination camera model plugin whose base type is image_reprojection::CameraModel
* other parameters for the destination camera model can be defined in the same namespace

**~dst_camera/info_file** (string, required)
* initial camera info of the destination camera will be loaded from this file by using camera_calibration_parsers::readCalibration()

**~dst_camera/frame_id** (string, default: camera name in the info file)
* frame id of the destination camera
* the camera name in the info file will be used as the default

**~dst_camera/encoding** (string, default: "bgr8")
* desired encoding of destination camera images
* conversion from encoding of source camera images to this encoding must be possible

**~dst_camera/fps** (double, default: 16.0)
* rate of destination camera image/info publishment

**~map_update/background** (bool, default: false)
* map update will be done in background if true, or in every reprojection of destination image if false
* try true when desired fps is not realized as map update is the most time consuming part in the algoritm

**~map_update/frequency** (double, default: 8.0)
* frequency of background map update
* affect only if ~map_update/background is true

**~map_update/binning_x** (int, default: 8)\
**~map_update/binning_y** (int, default: 8)
* scale of map between source images and a destination image
* size of map is defined by (width of dst image / binning_x, height of dst image / binning_y)
* try greater value (smaller map size) if cpu usage is too high although the quality of dst image may get lower

### Required tf Transforms
**\<dst camera frame> -> \<surface frame>**\
**\<surface frame> -> \<(every) src camera frame>**

* camera frame ids are obtained by image_reprojection::CameraModel::toCameraInfo()->header.frame_id
* surface frame ids are obtained by image_reprojection::SurfaceModel::getFrameId()

## Standard Camera Models

### PinholeCameraModel
* this is a wrapper of [opencv's pinhole camera model](https://docs.opencv.org/trunk/d9/d0c/group__calib3d.html)
* distortion model in camera info must be "plumb_bob" or "rational_polynomial"

### FisheyeCameraModel
* this is based on [opencv's fisheye camera model](https://docs.opencv.org/trunk/db/d58/group__calib3d__fisheye.html) but extended to support field of view over 180 degrees
* distortion model in camera info must be "fisheye"

#### Parameters
**~<camera_ns>/fov** (double, default: pi)
* field of view in radians

**~<camera_ns>/skew** (double, default: 0.0)
* skew coefficient of the distortion model

## Standard Surface Models

### PlaneSurfaceModel
* this implements a plane represented as [image_reprojection_plugins/PlaneStamped](image_reprojection_plugins/msg/PlaneStamped.msg)
* static_plane_publisher would be useful to periodically publish static plane messages

### SphereSurfaceModel
* this implements a sphere represented as [image_reprojection_plugins/SphereStamped](image_reprojection_plugins/msg/SphereStamped.msg)
* static_sphere_publisher would be useful to periodically publish static sphere messages

### MeshSurfaceModel
* this implements 3D mesh triangles represented as [image_reprojection_plugins/MeshStamped](image_reprojection_plugins/msg/MeshStamped.msg)
* static_mesh_publisher would be useful to periodically publish static mesh messages

### DEMSurfaceModel
* this implements digital elevation map (DEM) represented as nav_msgs/OccupancyGrid

#### Parameters
**~<surface_ns>/min_data** (double, default: 0.0)\
**~<surface_ns>/max_data** (double, default: 1.0)
* elevation value when nav_msgs::OccupancyGrid::data[] is 0 or 100
