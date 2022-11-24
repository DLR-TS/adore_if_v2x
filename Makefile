SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")
MAKEFILE_PATH:=$(shell dirname "$(abspath "$(lastword $(MAKEFILE_LIST)"))")

MAKEFLAGS += --no-print-directory

include coordinate_conversion/make_gadgets/make_gadgets.mk
include coordinate_conversion/make_gadgets/docker/docker-tools.mk

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

include adore_if_v2x.mk
include adore_if_ros_msg/adore_if_ros_msg.mk
include coordinate_conversion/coordinate_conversion.mk
include v2x_if_ros_msg/v2x_if_ros_msg.mk
include apt_cacher_ng_docker/apt_cacher_ng_docker.mk

.PHONY: all
all: build

.PHONY: set_env 
set_env: 
	$(eval PROJECT := ${ADORE_IF_V2X_PROJECT}) 
	$(eval TAG := ${ADORE_IF_V2X_TAG})


.PHONY: build
build: set_env start_apt_cacher_ng
	rm -rf ${ROOT_DIR}/${PROJECT}/build
	cd v2x_if_ros_msg && make 
	cd adore_if_ros_msg && make 
	cd plotlablib && make 
	cd coordinate_conversion && make 
	cd "${ROOT_DIR}" && \
    touch CATKIN_IGNORE
	docker build --network host \
                 --tag ${PROJECT}:${TAG} \
                 --build-arg PROJECT=${PROJECT} \
                 --build-arg ADORE_IF_ROS_MSG_TAG=${ADORE_IF_ROS_MSG_TAG} \
                 --build-arg V2X_IF_ROS_MSG_TAG=${V2X_IF_ROS_MSG_TAG} \
                 --build-arg PLOTLABLIB_TAG=${PLOTLABLIB_TAG} \
                 --build-arg COORDINATE_CONVERSION_TAG=${COORDINATE_CONVERSION_TAG} .
	cd "${ROOT_DIR}" && \
	docker cp $$(docker create --rm ${PROJECT}:${TAG}):/tmp/${PROJECT}/${PROJECT}/build "${ROOT_DIR}/${PROJECT}"

.PHONY: clean 
clean: 
	rm -rf ${ROOT_DIR}/${PROJECT}/build
	docker rm $$(docker ps -a -q --filter "ancestor=${PROJECT}:${TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${PROJECT}:${TAG}) 2> /dev/null || true
 
