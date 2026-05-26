# nvblox_ros 编译说明

本文记录本机编译 `nvblox_ros` 的推荐流程，以及每个步骤为什么需要这样做。

编译应当从 ROS 2 workspace 根目录执行：

```bash
cd ~/RealityLab/map_ws
```

不要在 `~/RealityLab/map_ws/src` 里执行 `colcon build`。如果在 `src` 下编译，
colcon 会生成 `src/build`、`src/install`、`src/log`，导致源码目录混入构建产物，
也会让后续 source 到错误的 workspace。

## 推荐编译命令

```bash
source /home/agxi/proxy_terminal.bashrc
source /home/agxi/miniconda3/etc/profile.d/conda.sh
conda deactivate
source /opt/ros/humble/setup.bash

export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}
export CUDACXX=/usr/local/cuda/bin/nvcc
export CMAKE_BUILD_PARALLEL_LEVEL=4
export MAKEFLAGS=-j4

colcon build --packages-select nvblox_ros \
  --parallel-workers 1 \
  --executor sequential \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=89 \
  -DBUILD_TESTING=OFF \
  -DBUILD_BENCHMARKS=OFF \
  -DBUILD_PYTORCH_WRAPPER=OFF \
  -DBUILD_RENDERER=OFF \
  -DUSE_SYSTEM_GFLAGS=ON \
  -DUSE_SYSTEM_GLOG=ON \
  -DUSE_SYSTEM_EIGEN=ON \
  -DUSE_SYSTEM_GTEST=ON
```

编译完成后，运行节点前 source workspace：

```bash
source ~/RealityLab/map_ws/install/setup.bash
```

## 为什么这样编译

### 从 workspace 根目录编译

期望的 ROS workspace 结构是：

```text
~/RealityLab/map_ws/
  build/
  install/
  log/
  src/
    nvblox/
    nvblox_ros/
```

`build`、`install`、`log` 应当和 `src` 同级，而不是放在 `src` 里面。这样源码目录
保持干净，`source ~/RealityLab/map_ws/install/setup.bash` 也会指向正确的工作空间。

如果误把构建产物生成到了 `src` 下，只删除这些生成目录即可：

```bash
rm -r ~/RealityLab/map_ws/src/build ~/RealityLab/map_ws/src/install ~/RealityLab/map_ws/src/log
```

### 先退出 conda

`colcon` 和 CMake 应当优先看到 ROS Humble 与系统环境。激活的 conda 环境可能把
自己的编译器、Python、CMake 包、动态库路径放到前面，导致 ROS 包发现、C++ 依赖
解析或链接行为变得很难判断。

所以命令里先 source conda shell integration，再执行：

```bash
conda deactivate
```

### 手动 source ROS Humble

`nvblox_ros` 依赖 ROS 2 Humble 环境中的 `ament_cmake`、`rclcpp`、`sensor_msgs`、
`tf2_msgs`、消息生成与 colcon 包发现机制：

```bash
source /opt/ros/humble/setup.bash
```

这个步骤放在退出 conda 之后，避免 ROS 路径被 conda 路径覆盖。

### 下载前 source proxy

如果 CMake 或依赖逻辑需要访问网络，先使用本机终端里的代理配置：

```bash
source /home/agxi/proxy_terminal.bashrc
```

这样下载行为和普通子终端保持一致。

### 显式指定 CUDA

`nvblox` 是 CUDA 工程。直接 `colcon build` 时可能出现：

```text
No CMAKE_CUDA_COMPILER could be found.
```

所以编译命令显式把 CUDA 放进环境，并把 `nvcc` 传给 CMake：

```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}
export CUDACXX=/usr/local/cuda/bin/nvcc
-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc
```

### 使用 RTX 4090 对应的 CUDA 架构

本机 GPU 是 RTX 4090，对应 Ada Lovelace，compute capability 为 8.9。因此使用：

```bash
-DCMAKE_CUDA_ARCHITECTURES=89
```

这样可以避免编译无关架构，也让生成的 CUDA 代码匹配本机 GPU。

### 只编译 nvblox_ros

推荐使用：

```bash
--packages-select nvblox_ros
```

当前 workspace 同时包含 `nvblox` 和 `nvblox_ros`。直接执行 `colcon build` 会尝试构建
所有包，可能先配置独立的 `nvblox` 包；如果 CUDA 环境没有准备好，就容易在这里失败。
选择 `nvblox_ros` 可以把构建范围限制在实际需要运行的 ROS 包及其依赖上。

### 并行度保持中等

CUDA 编译内存占用较高，并行开得太大会被系统 kill。当前推荐：

```bash
export CMAKE_BUILD_PARALLEL_LEVEL=4
export MAKEFLAGS=-j4
colcon build --parallel-workers 1 --executor sequential
```

这会让单个包内部保留一定并行度，同时避免多个重型包同时编译。

### 关闭当前不用的组件

日常运行 TSDF / Color TSDF ROS 节点不需要测试、benchmark、PyTorch wrapper 或 renderer：

```bash
-DBUILD_TESTING=OFF
-DBUILD_BENCHMARKS=OFF
-DBUILD_PYTORCH_WRAPPER=OFF
-DBUILD_RENDERER=OFF
```

关闭它们可以减少编译时间、依赖面和内存压力。

### 优先使用系统依赖

命令里还设置了：

```bash
-DUSE_SYSTEM_GFLAGS=ON
-DUSE_SYSTEM_GLOG=ON
-DUSE_SYSTEM_EIGEN=ON
-DUSE_SYSTEM_GTEST=ON
```

这样 CMake 会优先使用系统已经安装的依赖，尽量避免额外下载或本地构建依赖副本。

## 运行时配置文件

launch 文件已经调整为默认读取源码目录下的 YAML，而不是 `install/` 里的拷贝。

默认配置路径为：

```text
~/RealityLab/map_ws/src/nvblox_ros/config/galbot_home_head_front_left_tsdf.yaml
~/RealityLab/map_ws/src/nvblox_ros/config/galbot_home_head_front_left_color_tsdf.yaml
```

也就是说，日常修改 `nvblox_ros/config/` 下的 YAML 即可，不需要每次编译后再去改
`install/` 里的配置副本。

可以用下面命令确认 launch 默认参数：

```bash
source ~/RealityLab/map_ws/install/setup.bash
ros2 launch nvblox_ros tsdf_only.launch.py --show-args
ros2 launch nvblox_ros color_tsdf.launch.py --show-args
```

输出中的默认 `config_file` 应指向：

```text
~/RealityLab/map_ws/src/nvblox_ros/config/
```

## 常见错误

如果执行：

```bash
colcon build
```

报错：

```text
No CMAKE_CUDA_COMPILER could be found.
```

使用本文开头的推荐编译命令。关键点是：退出 conda、source ROS Humble、导出 CUDA
路径、设置 `CUDACXX`，并向 CMake 传入
`-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc`。
