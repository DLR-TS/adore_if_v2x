# This Makefile contains useful targets that can be included in downstream projects.

ifndef adore_if_v2x_MAKEFILE_PATH

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
ADORE_IF_V2X_PROJECT:=adore_if_v2x

ADORE_IF_V2X_MAKEFILE_PATH:=$(strip $(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")"))
ifeq ($(SUBMODULES_PATH),)
ADORE_IF_V2X_SUBMODULES_PATH:=${ADORE_IF_V2X_MAKEFILE_PATH}
else
ADORE_IF_V2X_SUBMODULES_PATH:=$(SUBMODULES_PATH)
endif

MAKE_GADGETS_PATH:=${ADORE_IF_V2X_SUBMODULES_PATH}/adore_if_ros_msg/make_gadgets
ifeq ($(wildcard $(MAKE_GADGETS_PATH)),)
$(info INFO: To clone submodules use: 'git submodules update --init --recursive')
$(info INFO: To specify alternative path for submodules use: SUBMODULES_PATH="<path to submodules>" make build')
$(info INFO: Default submodule path is: ${ADORE_IF_V2X_MAKEFILE_PATH}')
$(error "ERROR: ${MAKE_GADGETS_PATH} does not exist. Did you clone the submodules?")
endif
REPO_DIRECTORY:=${ADORE_IF_V2X_MAKEFILE_PATH}


ADORE_IF_V2X_TAG:=$(shell cd ${MAKE_GADGETS_PATH} && make get_sanitized_branch_name REPO_DIRECTORY=${REPO_DIRECTORY})

ADORE_IF_V2X_IMAGE:=${ADORE_IF_V2X_PROJECT}:${ADORE_IF_V2X_TAG}
ADORE_IF_V2X_IMAGE:=${ADORE_IF_V2X_IMAGE}

ADORE_IF_V2X_CMAKE_BUILD_PATH="${ADORE_IF_V2X_PROJECT}/build"

ADORE_IF_V2X_CMAKE_INSTALL_PATH="${ADORE_IF_V2X_CMAKE_BUILD_PATH}/install"

$(info ADORE_IF_V2X_SUBMODULES_PATH: ${ADORE_IF_V2X_SUBMODULES_PATH})
include ${MAKE_GADGETS_PATH}/make_gadgets.mk
include ${ADORE_IF_V2X_SUBMODULES_PATH}/plotlablib/plotlablib.mk
include ${ADORE_IF_V2X_SUBMODULES_PATH}/coordinate_conversion/coordinate_conversion.mk
include ${ADORE_IF_V2X_SUBMODULES_PATH}/v2x_if_ros_msg/v2x_if_ros_msg.mk
include ${ADORE_IF_V2X_SUBMODULES_PATH}/adore_if_ros_msg/adore_if_ros_msg.mk
include ${ADORE_IF_V2X_SUBMODULES_PATH}/v2x_if_ros_msg/v2x_if_ros_msg.mk

.PHONY: build_adore_if_v2x 
build_adore_if_v2x: ## Build adore_if_v2x
	cd "${ADORE_IF_V2X_MAKEFILE_PATH}" && make

.PHONY: clean_adore_if_v2x
clean_adore_if_v2x: ## Clean adore_if_v2x build artifacts
	cd "${ADORE_IF_V2X_MAKEFILE_PATH}" && make clean

.PHONY: branch_adore_if_v2x
branch_adore_if_v2x: ## Returns the current docker safe/sanitized branch for adore_if_v2x
	@printf "%s\n" ${ADORE_IF_V2X_TAG}

.PHONY: image_adore_if_v2x
image_adore_if_v2x: ## Returns the current docker image name for adore_if_v2x
	@printf "%s\n" ${ADORE_IF_V2X_IMAGE}

.PHONY: update_adore_if_v2x
update_adore_if_v2x:
	cd "${ADORE_IF_V2X_MAKEFILE_PATH}" && git pull
endif
