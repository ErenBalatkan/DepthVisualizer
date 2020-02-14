# DepthVisualizer
A simple OpenGl based Rendering library for visualizing Lidar Point Clouds & Depth Maps.

![Sample Gif](https://i.imgur.com/X19k7PF.gif)

#### Installation
```
pip install DepthVisualizer
```
#### Supported Features:
- [x] Converting depth maps to point clouds
- [x] Converting point clouds to voxel maps
- [x] Visualizing 3D & 2D Bounding boxes
- [x] Keyboard Controls
- [x] Extracting rendered frame as numpy array
- [x] Updating point cloud data real-time
- [x] Methods for working with Kitti Dataset

#### Examples
Please refer to following repository for usage examples
https://github.com/Navhkrin/DepthVisualizer-Examples

#### Keyboard Controls
| Key        | Action           |
|:-------------:|:-------------:|
| W            | Move Forward |
| A            |  Move Left      |
| S            | Move Right      |
| D            | Move Back       |
| Left-Shift   | Move Up (+z)    |
| Left-Control | Move Down (-z)  |
| Arrow Up     | Turn Up        |
| Arrow Left  | Turn Left        |
| Arrow Right   | Turn Right       |
| Arrow Down   | Turn Down   |
| 1 | Enable Point Rendering   |
| 2 | Disable Point Rendering      |
| 3 | Enable Line Rendering      |
| 4 | Disable Line Rendering      |
| 5 | Enable Voxel Rendering      |
| 6 | Disable Voxel Rendering      |
