# This Makefile contains useful targets that can be included in downstream projects.

ifndef adore_if_v2x_MAKEFILE_PATH

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
adore_if_v2x_project:=adore_if_v2x
ADORE_IF_V2X_PROJECT:=${adore_if_v2x_project}

adore_if_v2x_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
make_gadgets_PATH:=${adore_if_v2x_MAKEFILE_PATH}/adore_if_ros_msg/make_gadgets
REPO_DIRECTORY:=${adore_if_v2x_MAKEFILE_PATH}

adore_if_v2x_tag:=$(shell cd ${make_gadgets_PATH} && make get_sanitized_branch_name REPO_DIRECTORY=${REPO_DIRECTORY})
ADORE_IF_V2X_TAG:=${adore_if_v2x_tag}

adore_if_v2x_image:=${adore_if_v2x_project}:${adore_if_v2x_tag}
ADORE_IF_V2X_IMAGE:=${adore_if_v2x_image}

adore_if_v2x_CMAKE_BUILD_PATH="${adore_if_v2x_project}/build"
ADORE_IF_V2X_CMAKE_BUILD_PATH=${adore_if_v2x_CMAKE_BULID_PATH}!

adore_if_v2x_CMAKE_INSTALL_PATH="${adore_if_v2x_CMAKE_BUILD_PATH}/install"
ADORE_IF_V2X_CMAKE_INSTALL_PATH=${adore_if_v2x_CMAKE_INSTALL_PATH}

include ${adore_if_v2x_MAKEFILE_PATH}/plotlablib/plotlablib.mk
include ${adore_if_v2x_MAKEFILE_PATH}/coordinate_conversion/coordinate_conversion.mk
include ${adore_if_v2x_MAKEFILE_PATH}/v2x_if_ros_msg/v2x_if_ros_msg.mk
include ${adore_if_v2x_MAKEFILE_PATH}/adore_if_ros_msg/adore_if_ros_msg.mk
include ${adore_if_v2x_MAKEFILE_PATH}/v2x_if_ros_msg/v2x_if_ros_msg.mk



.PHONY: build_adore_if_v2x 
build_adore_if_v2x: ## Build adore_if_v2x
	cd "${adore_if_v2x_MAKEFILE_PATH}" && make

.PHONY: clean_adore_if_v2x
clean_adore_if_v2x: ## Clean adore_if_v2x build artifacts
	cd "${adore_if_v2x_MAKEFILE_PATH}" && make clean

.PHONY: branch_adore_if_v2x
branch_adore_if_v2x: ## Returns the current docker safe/sanitized branch for adore_if_v2x
	@printf "%s\n" ${adore_if_v2x_tag}

.PHONY: image_adore_if_v2x
image_adore_if_v2x: ## Returns the current docker image name for adore_if_v2x
	@printf "%s\n" ${adore_if_v2x_image}

.PHONY: update_adore_if_v2x
update_adore_if_v2x:
	cd "${adore_if_v2x_MAKEFILE_PATH}" && git pull
endif
