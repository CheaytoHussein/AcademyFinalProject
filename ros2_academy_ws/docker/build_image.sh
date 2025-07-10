#!/bin/bash

# Used for building docker image on cross platform
# docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

DOCKER_BUILDKIT=1 docker build \
  --network host \
  --build-arg ros_distro=humble \
  --secret id=ssh-priv,src=$HOME/.ssh/id_rsa \
  --secret id=ssh-known-hosts,src=$HOME/.ssh/known_hosts \
  -t academy_gazebo_ros2:latest  \
  -f docker/dockerfile $@ .