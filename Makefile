SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")

MAKEFLAGS += --no-print-directory

include adore_if_v2x.mk
include ${ADORE_IF_V2X_SUBMODULES_PATH}/ci_teststand/ci_teststand.mk

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG:=

.PHONY: all
all: help

.PHONY: set_env 
set_env: 
	$(eval PROJECT := ${ADORE_IF_V2X_PROJECT}) 
	$(eval TAG := ${ADORE_IF_V2X_TAG})

.PHONY: build
build: set_env start_apt_cacher_ng build_adore_if_ros_msg build_v2x_if_ros_msg build_plotlablib build_coordinate_conversion
	rm -rf ${ROOT_DIR}/${PROJECT}/build
	cd "${ROOT_DIR}" && \
    touch CATKIN_IGNORE
	docker build --network host \
                 --tag ${PROJECT}:${TAG} \
                 --build-arg PROJECT=${PROJECT} \
                 --build-arg ADORE_IF_ROS_MSG_TAG=${ADORE_IF_ROS_MSG_TAG} \
				 --build-arg ADORE_IF_ROS_TAG=${ADORE_IF_ROS_TAG} \
                 --build-arg V2X_IF_ROS_MSG_TAG=${V2X_IF_ROS_MSG_TAG} \
                 --build-arg PLOTLABLIB_TAG=${PLOTLABLIB_TAG} \
				 --build-arg LIBADORE_TAG=${LIBADORE_TAG} \
                 --build-arg COORDINATE_CONVERSION_TAG=${COORDINATE_CONVERSION_TAG} .
	cd "${ROOT_DIR}" && \
	docker cp $$(docker create --rm ${PROJECT}:${TAG}):/tmp/${PROJECT}/${PROJECT}/build "${ROOT_DIR}/${PROJECT}"

.PHONY: clean_submodules
clean_submodules: clean_adore_if_ros_msg clean_plotlablib clean_coordinate_conversion clean_v2x_if_ros_msg

.PHONY: clean 
clean: set_env clean_submodules 
	rm -rf ${ROOT_DIR}/${PROJECT}/build
	docker rm $$(docker ps -a -q --filter "ancestor=${PROJECT}:${TAG}") --force 2> /dev/null || true
	docker rmi $$(docker images -q ${PROJECT}:${TAG}) --force 2> /dev/null || true
 
.PHONY: test
test: ci_test
