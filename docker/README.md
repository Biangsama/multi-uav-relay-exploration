# Docker Deployment

这套 Docker 文件用于把仓库直接作为 Catkin `src/` 树打进镜像，并部署到 Ubuntu 22.04 主机上的 Docker 环境。

容器基座固定为：

- ROS Noetic
- Ubuntu 20.04 (focal)

这是因为当前代码是 ROS1 Noetic / Catkin 工程；在 Ubuntu 22.04 主机上通过 Docker 运行 focal + noetic 是最稳的方案。

## 仓库结构约定

这个仓库根目录本身就是 Catkin 工作区的 `src/` 目录内容。

Docker 构建时会自动在容器内创建：

- `/opt/relay_explore_ws`
- `/opt/relay_explore_ws/src`

然后把当前仓库复制到 `/opt/relay_explore_ws/src` 下再执行 `catkin_make`。

## 构建镜像

```bash
docker compose build relay-explore
```

或者：

```bash
./docker/build.sh
```

如果机器内存较小，可以这样限制并行度：

```bash
CATKIN_JOBS=2 NS3_JOBS=2 docker compose build relay-explore
```

## ns-3 说明

默认会在镜像构建时自动拉取并编译 `ns-3.38`，安装到：

- `/opt/relay_explore_ws/.deps/ns3.42-install`

这样可以直接满足当前 `ns3_connector` 的查找路径。

如果你只想先把非 `ns3_connector` 的部分编译出来，可以关闭它：

```bash
BUILD_NS3=0 docker compose build relay-explore
```

但关闭后，涉及 `basic_wifi_adhoc_with_controller` 的 launch 不能正常运行。

## 进入容器

```bash
./docker/run.sh
```

也可以直接执行命令：

```bash
./docker/run.sh rospack find relay_racer_integration
```

## RViz / X11

如果需要在主机上显示 RViz，主机需要运行图形桌面和 X11。

先在主机执行：

```bash
xhost +local:root
```

然后运行：

```bash
./docker/run.sh roslaunch relay_racer_integration relay_rviz.launch
```

脚本会自动挂载 `/tmp/.X11-unix`。

如果主机是无头环境，就不要启动 RViz。

## 运行 6 机实验

```bash
./docker/run.sh roslaunch relay_racer_integration relay_real_input_6planner_min.launch
```

## 依赖处理

Dockerfile 会执行：

- `rosdep install --from-paths src --ignore-src`
- `catkin_make`

同时跳过这几个仓库里遗留但当前主链路不需要的旧依赖键：

- `svo_msgs`
- `vikit_ros`
- `swarmtal_msgs`
