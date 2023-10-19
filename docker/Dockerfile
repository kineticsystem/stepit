# Install Ubuntu 20.04 with user "developer".

FROM osrf/ros:humble-desktop-full-jammy as ros

LABEL name="develop:ros"

RUN apt-get update
RUN apt-get -y install sudo
RUN echo 'root:docker' | chpasswd                      # Update user root password.
RUN useradd --create-home --shell /bin/bash developer  # Create the user developer.
RUN echo 'developer:developer' | chpasswd              # Update user developer password.
RUN usermod -aG sudo developer                         # Add user developer to sudoers.

RUN apt-get -y upgrade
RUN apt-get -y install vim wget unzip python3
RUN apt-get -y install jstest-gtk # Test connected joysticks.
RUN apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp

CMD ["bash"]

USER developer

RUN mkdir -p /home/developer/.ros
COPY --chown=developer:developer cyclonedds.xml /home/developer/.ros

ENV HOME /home/developer
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
RUN echo "export CYCLONEDDS_URI=~/.ros/cyclonedds.xml" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN rosdep update

WORKDIR /home/developer/repo/stepit