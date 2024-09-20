FROM osrf/ros:iron-desktop-full

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

CMD ["tail", "-f", "/dev/null"]