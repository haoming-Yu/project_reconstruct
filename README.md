# Project Reconstruct
1. project description: this project supports input of RGB(not supported now), Depth, and IMU(not supported now).
2. the project aims to reconstruct a mesh with color textures.
3. the project is being developed, after completion, the hardware setting will be open-sourced.

# TODO
1. Use IR value to thresholding depth value, only trust depth value with specific thresholding of IR value (because of the auto exposure's existance, the thresholding need to dynamically select pixels with high trust range.) **Done, but thresholding is static for now**
2. 2D edge filter by clear away edges' inaccuracy by finding wrong pixel (sensor-level mask & fast changing of depth) and reset the depth to some special value for 3D detection. **Done, use various 2D filter including gradient filter**
3. 3D edge filter by calculate normal and filter away the normal problem, including pca-based normal calculation. But following current pipeline, we can not get neighboring points added by former measurement at surface measurement calculation time. Thus, we keep the former simple calculation of surface measurement. **Done, but PCA needs CPU calculation, which is difficult to change. Thus use vanilla method for normal calculation.**
4. set the camera into a ROS node and add voxel hashing. **Processing, ROS is difficult to integrate for we have to rewrite the driver of ROS1, thus here we first process voxel hashing.**