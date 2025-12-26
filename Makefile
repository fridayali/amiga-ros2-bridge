IMAGE:=ghcr.io/ucmercedrobotics/amiga-ros2-bridge
WORKSPACE:=amiga-ros2-bridge
NOVNC:=ghcr.io/ucmercedrobotics/docker-novnc
MACHINE_NAME?=agx
.ONESHELL:
.SHELL := /bin/bash


PORT:=12346
PAYLOAD:=true
ARCH := $(shell uname -m)
PLATFORM := linux/amd64
TARGET:=base
ARCH_TAG:=x86_64
CUDA_MOUNT:=
ifneq (,$(filter $(ARCH),arm64 aarch64))
	PLATFORM := linux/arm64/v8
	ARCH_TAG:=arm64
	TARGET:=jetson
	CUDA_MOUNT:= --runtime=nvidia \
			 -v /usr/local/cuda:/usr/local/cuda:ro \
			 --device /dev/nvhost-gpu \
			 --device /dev/nvmap 
endif

repo-init:
	python3 -m pip install pre-commit && \
	pre-commit install

shell:
	CONTAINER_PS=$(shell docker ps -aq --filter ancestor=${IMAGE}:${ARCH_TAG}) && \
	docker exec -it $${CONTAINER_PS} bash

build-image:
	docker build --platform ${PLATFORM} . -t ${IMAGE}:${ARCH_TAG} --target ${TARGET}

vnc:
	docker run -d --rm --net=host \
	--name=novnc \
	${NOVNC}

udev:
	cp udev/99-ucm.rules /etc/udev/rules.d && \
	udevadm control --reload-rules && \
	udevadm trigger

bash: udev
	docker run -it --rm \
	--net=host \
	--privileged \
	${CUDA_MOUNT} \
	--env="DISPLAY=:2" \
	-v .:/${WORKSPACE}:Z \
	-v ~/.ssh:/root/.ssh:ro \
	-v /dev/:/dev/ \
	-e FASTDDS_DEFAULT_PROFILE_FILE=file:///${WORKSPACE}/dds/${MACHINE_NAME}.xml \
	${IMAGE}:${ARCH_TAG} bash

deps:
	rosdep install --from-paths . --ignore-src -r -y

clean:
	rm -rf build/ install/ log/

bringup:
	ros2 launch amiga_bringup brain_bringup.launch.py

amiga-streams:
	PYTHONPATH=/.venv/lib/python3.10/site-packages:$$PYTHONPATH \
	ros2 launch amiga_ros2_bridge amiga_streams.launch.py
.PHONY: amiga-webscoket

amiga-websocket:
	set -e
	echo "[1/5] Setting PYTHONPATH..."
	export PYTHONPATH=/.venv/lib/python3.10/site-packages:$$PYTHONPATH

	echo "[2/5] Waiting 2 seconds"
	sleep 2

	echo "[3/5] Launching amiga_streams..."
	ros2 launch amiga_ros2_bridge amiga_streams.launch.py &
	STREAMS_PID=$$!

	echo "[4/5] Waiting 3 seconds"
	sleep 3

	echo "[5/5] Starting path_follower and websocket..."
	cd amiga-ros2-bridge/amiga_ros2_bridge/amiga_ros2_bridge

	python3 path_follower.py &
	PATH_FOLLOWER_PID=$$!

	python3 websocket.py &
	WEBSOCKET_PID=$$!

	echo "amiga_streams PID      : $$STREAMS_PID"
	echo "path_follower PID      : $$PATH_FOLLOWER_PID"
	echo "websocket PID          : $$WEBSOCKET_PID"

	wait
twist:
	ros2 launch amiga_ros2_bridge twist_control.launch.py

joy:
	ros2 launch amiga_ros2_teleop joy.launch.py

foxglove:
	ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8766

oakd:
	ros2 launch amiga_ros2_oakd amiga_cameras.launch.py

description:
	ros2 launch amiga_ros2_description urdf.launch.py

localization:
	ros2 launch amiga_localization bringup.launch.py

mission-interface:
	ros2 run amiga_ros2_behavior_tree bt_runner --ros-args -p mission_port:=${PORT} -p mission_payload_length_included:=${PAYLOAD}

amiga:
	./scripts/bringup_amiga_tmux.sh

cartographer:
	apt update && \
	apt install -y \
	ros-$(ROS_DISTRO)-cartographer \
	ros-$(ROS_DISTRO)-cartographer-ros

build-prod:
	docker buildx build --platform linux/arm64/v8 . -t ${IMAGE} --target base

