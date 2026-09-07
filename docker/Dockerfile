# Install Ubuntu 24.04 with user "developer".

FROM osrf/ros:jazzy-desktop-full AS ros

LABEL name="develop:ros"

# Fix the expired ROS GPG key issue by bypassing signature verification
RUN echo 'APT::Get::AllowUnauthenticated "true";' > /etc/apt/apt.conf.d/99allow-unauth-ros
RUN echo 'Acquire::AllowInsecureRepositories "true";' >> /etc/apt/apt.conf.d/99allow-unauth-ros
RUN apt-get update

RUN apt-get -y install sudo
RUN echo 'root:docker' | chpasswd                      # Update user root password.

# Ubuntu 24.04 base images ship a default 'ubuntu' user at uid/gid 1000. Remove
# it so 'developer' can be created with a matching uid/gid to the host user
# (passed in via build args), which keeps the bind-mounted workspace writable.
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi
RUN if getent group ubuntu >/dev/null 2>&1; then groupdel ubuntu; fi

RUN groupadd -g 107 input                              # Create the group 'input' (id 107) to access devices.

ARG USER_UID=1000
ARG USER_GID=1000

# Create the user developer with a uid/gid matching the host user so files
# created in the bind-mounted workspace (e.g. colcon's build/install/log
# directories) are writable and owned correctly on the host.
RUN groupadd -g ${USER_GID} developer
RUN useradd --create-home --shell /bin/bash --uid ${USER_UID} --gid ${USER_GID} developer
RUN echo 'developer:developer' | chpasswd              # Update user developer password.
RUN usermod -aG sudo,input developer                   # Add user to sudoers and input devices.

RUN apt-get -y upgrade
RUN apt-get -y install wget
RUN apt-get -y install unzip
RUN apt-get -y install python3
RUN apt-get -y install python-is-python3
RUN apt-get -y install jstest-gtk                      # Test connected joysticks.
RUN apt-get install -y --no-install-recommends ros-jazzy-rmw-cyclonedds-cpp
RUN apt-get install -y python3-colcon-common-extensions

CMD ["bash"]

USER developer

RUN mkdir -p /home/developer/.ros
COPY --chown=developer:developer cyclonedds.xml /home/developer/.ros

ENV HOME=/home/developer
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
RUN echo "export CYCLONEDDS_URI=~/.ros/cyclonedds.xml" >> ~/.bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN rosdep update

WORKDIR /home/developer/ws
