# Camera: Orbbec Femto Bolt

[launch/camera.launch.py](launch/camera.launch.py) starts the camera the same way as the people tracker's [jetson_pipeline.sh](../../autonomy/perception/social_perception/jetson_pipeline.sh), so both use one camera setup:

1. the Orbbec ROS 2 driver (`ros2 launch orbbec_camera femto_bolt.launch.py depth_registration:=false`),
2. depth registration as its own node (`depth_image_proc register_node`), which aligns depth to colour.

If either exits, the launch stops, as in the pipeline. It needs the `SOCIAL=true` image (the default), which has both packages.

`driver:=sdk` starts the old [camera.py](camera.py) instead: one node that reads the camera through the Orbbec SDK (`pyorbbecsdk2`, in the `PETE=true` image). The table below is what `camera.py` publishes; with the Orbbec driver the first three topics are the same, and the point cloud and the small preview are not published.

| Topic | Type | |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/Image` | colour, `bgr8`, 1280×720 at about 18 fps on the Jetson |
| `/camera/depth_registered/image_rect` | `sensor_msgs/Image` | depth aligned to colour, `16UC1` in mm, 0 = no reading |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | colour intrinsics |
| `/camera/points` | `sensor_msgs/PointCloud2` | XYZ points in metres, published when subscribed |
| `/camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | small 640 px jpeg preview, for watching from another computer |

All are best effort. Colour and depth of a frame share the same timestamp. Perception ([autonomy/perception](../../autonomy/perception)) and the people tracker both use these topics. Only one program can open the camera: do not run this launch file together with `jetson_pipeline.sh`, which starts the same driver itself.

## Run

The container needs the USB camera and the GPU (the SDK's depth engine runs on it):

```bash
docker run --rm -it --network=host --ipc=host --runtime nvidia --gpus all \
  --privileged -v /dev:/dev -v "$PWD:/workspace" navigoat:dev bash
ros2 launch /workspace/ros/src/hardware/camera/launch/camera.launch.py
```

Or start it together with perception: `ros2 launch /workspace/ros/src/autonomy/perception/launch/perception.launch.py camera:=true`.

## Watch it

On a laptop on the same network (it needs ROS 2 Jazzy and OpenCV), `python3 view_camera.py` opens a window with the preview. Press `q` to quit. Full-size frames don't get through a USB or Wi-Fi link, which is why it shows the jpeg preview.
