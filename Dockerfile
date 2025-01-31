# 使用官方的ros镜像，如果需要其他版本，在此更改
# 如果拉取失败，可以尝试更换源/代理，或者先手动拉取
FROM docker-mirror.aigc2d.com/ros:jazzy

# 设置工作目录
WORKDIR /ros_test_ws

# 换源 按需更改
RUN sed -i 's/ports.ubuntu.com/mirrors.seu.edu.cn/g' /etc/apt/sources.list.d/ubuntu.sources

# 更新并安装构建依赖项，没有OpenSSL会报错
RUN apt-get update 
RUN apt-get install -y \
   libssl-dev ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-controller-interface ros-jazzy-controller-manager liblog4cpp5v5

RUN apt-get upgrade -y

# 这里按需更改，rosdep相关
# COPY ./src/ros2_control_demos /home/ros2_ws/src/ros2_control_demos

# 这里按需更改
ENV HTTP_PROXY=10.208.95.154:20171
ENV HTTPS_PROXY=10.208.95.154:20171

ENV http_proxy=10.208.95.154:20171
ENV https_proxy=10.208.95.154:20171

# rosdep相关，虚拟机里运行十分缓慢，谨慎使用，这里按需更改
# RUN cd /home/ros2_ws/src \
#    && rosdep update --rosdistro ${ROS_DISTRO}  

# RUN rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO} \
#    --skip-keys ros-${ROS_DISTRO}-joint-state-publisher-gui --skip-keys ros-${ROS_DISTRO}-rviz2\
#    && \
#    : "remove cache" && \
#    apt-get autoremove -y -qq && \
#    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/bin/bash", "-c"]
