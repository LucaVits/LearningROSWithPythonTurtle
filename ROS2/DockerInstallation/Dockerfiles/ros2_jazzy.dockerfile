FROM osrf/ros:jazzy-desktop-full

RUN apt-get update && apt-get install -y \
    python3-ipykernel \
    python3-pickleshare \
    wget \
    && rm -rf /var/lib/apt/lists/*

ENV HOME=/home/ubuntu
ENV WORKSPACE_DIR=${HOME}/learn_ros_ws

WORKDIR ${WORKSPACE_DIR}

# Generate generic build files. 
# We do this now so setup.bash is generated with our base underlay.
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# On every new terminal launch, source the workspace.
RUN echo "source ${WORKSPACE_DIR}/install/setup.bash" >> ~/.bashrc

COPY NotebookTutorials NotebookTutorials

CMD ["tail", "-f", "/dev/null"]