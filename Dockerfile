FROM thbarkouki/homestri-ur:latest

SHELL ["/bin/bash", "-c"]

# Install python dependencies
# RUN pip3 install \ 

# Install ROS dependencies
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     && rm -rf /var/lib/apt/lists/*

# Set the working directory in the container
WORKDIR /root/catkin_ws

# Copy the 'src' directory from 'catkin_ws' to the container's workspace
COPY . ./src/homestri_explanation_bts

RUN source /opt/ros/noetic/setup.bash && \
    apt-get update && rosdep install -q -y \
      --from-paths ./src \
      --ignore-src \
      --rosdistro noetic \
    && rm -rf /var/lib/apt/lists/*

# Build the ROS workspace
RUN source /opt/ros/noetic/setup.bash && \
    catkin build