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

# The container user is in the sudo group, but a password prompt makes the
# scripts in ./bin unusable from a non-interactive shell: rosdep shells out to
# `sudo apt-get`, and sudo refuses to read a password without a terminal. That
# is what would break `docker exec <container-name> update.sh`.
RUN echo 'developer ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/developer \
 && chmod 0440 /etc/sudoers.d/developer

CMD ["bash"]

USER developer

RUN mkdir -p /home/developer/.ros
COPY --chown=developer:developer cyclonedds.xml /home/developer/.ros

ENV HOME=/home/developer

# The DDS configuration. These are ENV rather than exports in the bashrc so that
# they also apply to non-interactive commands, e.g.
# `docker exec <container-name> ros2 topic list`. Note the path has to be
# spelled out: nothing expands a ~ in an ENV.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/home/developer/.ros/cyclonedds.xml

# The repo is bind-mounted at ~/ws (see docker-compose.yml), so ./bin lives at
# a fixed path inside the container. Putting it on the PATH here rather than in
# the bashrc means it also applies to non-interactive shells, e.g.
# `docker exec <container-name> build.sh`.
ENV PATH="/home/developer/ws/bin:${PATH}"

# Whatever genuinely needs an interactive shell -- sourcing the ROS2 setup, the
# aliases -- lives in this file, which is versioned in the repo instead of
# being echoed line by line into ~/.bashrc. Source it from the stock ~/.bashrc
# rather than replacing that file, to keep the distribution's own settings.
COPY --chown=developer:developer bashrc /home/developer/.bashrc.stepit
RUN echo "source ~/.bashrc.stepit" >> ~/.bashrc

RUN rosdep update

WORKDIR /home/developer/ws
