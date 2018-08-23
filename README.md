# image-reprojection
A ROS nodelet to reproject image as if viewing from different viewpoint

## Reprojection Algorithm
1. Calculate mapping between source and distination image pixels
    1. Project destination image pixels to 3D rays by using a **destination camera model**
    1. Calculate intersections between 3D rays from the destination camera and a surface by using a **surface model**
    1. Project intersection points to source image pixels by using a **source camera model**
1. Generate a destination image by mapping source to destination image pixels

* Models used in 1-1 to 1-3 are implemented as plugins so user can use own models as well as standard models described below
* 1 and 2 can be performed either sequentially or simultaneously

## Nodelet: ImageReprojection

### Subsribed topics
src_imageN <N = 0, 1, ..> (sensor_msgs/Image)
* subscribed if the parameter src_cameraN/model is defined
* sensor_msgs/CameraInfo will also be subscribed by using image_transport::CameraSubscriber

surface (arbitrary type)
* type must be supported by surface model specified by the parameter ~surface/model

### Published topics
dst_image (sensor_msgs/Image)
* sensor_msgs/CameraInfo will also be published by using image_transport::CameraPublisher

### Services
set_dst_camera_info (sensor_msgs/SetCameraInfo)

### Parameters
~src_cameraN/model <N = 1, 2, ..> (string, required for N = 0)
* name or type of source camera model plugin whose base type is image_reprojection::CameraModel
* other parameters for source camera model can be defined in the same namespace

~surface/model (string, required)
* name or type of surface model plugin whose base type is image_reprojection::SurfaceModel
* other parameters for surface model can be defined in the same namespace

~dst_camera/model (string, required)
* name or type of destination camera model plugin whose base type is image_reprojection::CameraModel
* other parameters for destination camera model can be defined in the same namespace

~dst_camera/info_file (string, required)
* initial camera info of destination camera will be loaded from this file by using camera_calibration_parsers::readCalibration()

~dst_camera/encoding (string, default: "bgr8")
* desired encoding of destination camera image
* conversion from encoding of source camera image to this encoding must be possible

~dst_camera/fps (double, default: 16.0)
* rate of destination camera image/info publishment

~map_update/background (bool, default: false)
* map update will be done in background if true, or in every reprojection of destination image if false
* try true when desired fps is not realized as map update is most time consuming part of the algoritm

~map_update/frequency (double, default: 8.0)
* frequency of background map update
* affect only if ~map_update/background is true

~map_update/binning_x (int, default: 8)\
~map_update/binning_y (int, default: 8)
* scale of map between source images and destination image
* size of map is defined by (width of dst image / binning_x, height of dst image / binning_y)
* try greater value (smaller map size) if cpu usage is too high although the quality of dst image gets lower

## Standard Camera Models

### PinholeCameraModel
To be described ...

### FisheyeCameraModel
To be described ...

## Standard Surface Models

### MeshSurfaceModel
To be described ...

### DEMSurfaceModel
Not available in this version ...
