# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|
  config.vm.box = "ubuntu/focal64"
  config.vm.box_check_update = false
  config.vm.hostname = "noetic-focal"

  config.vm.provider "virtualbox" do |vb|
    vb.name = "noetic-focal"
    vb.gui = true
    vb.memory = "4096"
    vb.cpus = 3
  end

  config.vm.provision "shell", inline: <<-SHELL
    # Install Ubuntu Desktop
    sudo apt-get update && sudo apt-get upgrade -y
    sudo apt-get install -y --no-install-recommends ubuntu-desktop
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends gdm3
    sudo bash -c "echo '/usr/sbin/gdm3' > /etc/X11/default-display-manage"
    sudo DEBIAN_FRONTEND=noninteractive DEBCONF_NONINTERACTIVE_SEEN=true dpkg-reconfigure gdm3
    sudo bash -c "echo set shared/default-x-display-manager gdm3 | debconf-communicate"
    sudo apt-get install -y --no-install-recommends virtualbox-guest-dkms virtualbox-guest-utils virtualbox-guest-x11
    sudo usermod -a -G sudo vagrant
    sudo apt-get install -y ttf-ubuntu-font-family

    # Install ROS Noetic Desktop Full
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt-get update
    sudo apt-get install -y ros-noetic-desktop-full python3-osrf-pycommon python3-catkin-tools
    sudo apt-get install -y python3-pip python3-rosdep
    sudo rosdep init && rosdep update
    echo "source /opt/ros/noetic/setup.bash" >> /home/vagrant/.bashrc
  SHELL
end
