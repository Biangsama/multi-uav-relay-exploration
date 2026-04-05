FROM osrf/ros:noetic-desktop-full-focal

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic
ARG CATKIN_WS=/opt/relay_explore_ws
ARG CATKIN_JOBS=4
ARG NS3_VERSION=ns-3.38
ARG NS3_JOBS=4
ARG BUILD_NS3=1

ENV DEBIAN_FRONTEND=${DEBIAN_FRONTEND}
ENV ROS_DISTRO=${ROS_DISTRO}
ENV CATKIN_WS=${CATKIN_WS}
ENV QT_X11_NO_MITSHM=1
ENV PYTHONDONTWRITEBYTECODE=1

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends     build-essential     ca-certificates     cmake     git     libarmadillo-dev     libboost-all-dev     libeigen3-dev     libopencv-dev     libpcl-dev     libprotobuf-dev     libyaml-cpp-dev     pkg-config     protobuf-compiler     python3-numpy     python3-pip     python3-rosdep     python3-yaml     wget     x11-apps     && rm -rf /var/lib/apt/lists/*

RUN if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then       rosdep init;     fi     && rosdep update --rosdistro ${ROS_DISTRO}

WORKDIR ${CATKIN_WS}
RUN mkdir -p src

COPY . ${CATKIN_WS}/src
COPY docker/entrypoint.sh /ros_entrypoint_workspace.sh

RUN chmod +x /ros_entrypoint_workspace.sh

RUN if [ "${BUILD_NS3}" = "1" ]; then       git clone --branch ${NS3_VERSION} --depth 1 https://gitlab.com/nsnam/ns-3-dev.git /tmp/ns-3;       cmake -S /tmp/ns-3 -B /tmp/ns-3/build         -DCMAKE_BUILD_TYPE=Release         -DCMAKE_INSTALL_PREFIX=${CATKIN_WS}/.deps/ns3.42-install         -DNS3_EXAMPLES=OFF         -DNS3_TESTS=OFF;       cmake --build /tmp/ns-3/build -j${NS3_JOBS};       cmake --install /tmp/ns-3/build;       rm -rf /tmp/ns-3;     else       mkdir -p ${CATKIN_WS}/.deps/ns3.42-install;     fi

ENV NS3_DIR=${CATKIN_WS}/.deps/ns3.42-install/lib/cmake/ns3

RUN source /opt/ros/${ROS_DISTRO}/setup.bash     && rosdep install          --rosdistro ${ROS_DISTRO}          --from-paths src          --ignore-src          -r -y          --skip-keys="svo_msgs vikit_ros swarmtal_msgs"     && catkin_make          -DCMAKE_BUILD_TYPE=Release          -j${CATKIN_JOBS}          -l${CATKIN_JOBS}

ENTRYPOINT ["/ros_entrypoint_workspace.sh"]
CMD ["bash"]
