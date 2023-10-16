# Install Ubuntu 20.04 with user "developer".

############################## build ros ##############################

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

CMD ["bash"]

USER developer

ENV HOME /home/developer
RUN echo "source /opt/ros/humble/setup.bash" >> /home/developer/.bashrc
RUN rosdep update

WORKDIR /home/developer/repo/stepit