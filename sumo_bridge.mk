# This Makefile contains useful targets that can be included in downstream projects.

ifeq ($(filter sumo_if_ros.mk, $(notdir $(MAKEFILE_LIST))), sumo_if_ros.mk)

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
SUMO_IF_ROS_PROJECT:=sumo_if_ros

SUMO_IF_ROS_MAKEFILE_PATH:=$(strip $(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")"))
ifeq ($(SUBMODULES_PATH),)
    SUMO_IF_ROS_SUBMODULES_PATH:=${SUMO_IF_ROS_MAKEFILE_PATH}
else
    SUMO_IF_ROS_SUBMODULES_PATH:=$(shell realpath ${SUBMODULES_PATH})
endif

MAKE_GADGETS_PATH:=${SUMO_IF_ROS_SUBMODULES_PATH}/make_gadgets
ifeq ($(wildcard $(MAKE_GADGETS_PATH)/*),)
    $(info INFO: To clone submodules use: 'git submodules update --init --recursive')
    $(info INFO: To specify alternative path for submodules use: SUBMODULES_PATH="<path to submodules>" make build')
    $(info INFO: Default submodule path is: ${SUMO_IF_ROS_MAKEFILE_PATH}')
    $(error "ERROR: ${MAKE_GADGETS_PATH} does not exist. Did you clone the submodules?")
endif
APT_CACHER_NG_DOCKER_PATH:=${SUMO_IF_ROS_SUBMODULES_PATH}/apt_cacher_ng_docker
SUMO_IF_ROS_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
ifeq ($(SUBMODULES_PATH),)
    SUMO_IF_ROS_SUBMODULES_PATH:=${SUMO_IF_ROS_MAKEFILE_PATH}
else
    SUMO_IF_ROS_SUBMODULES_PATH:=$(shell realpath ${SUBMODULES_PATH})
endif
REPO_DIRECTORY:=${SUMO_IF_ROS_MAKEFILE_PATH}

SUMO_IF_ROS_TAG:=$(shell cd "${MAKE_GADGETS_PATH}" && make get_sanitized_branch_name REPO_DIRECTORY="${REPO_DIRECTORY}")
SUMO_IF_ROS_IMAGE:=${SUMO_IF_ROS_PROJECT}:${SUMO_IF_ROS_TAG}

SUMO_IF_ROS_CMAKE_BUILD_PATH:="${SUMO_IF_ROS_PROJECT}/build"
SUMO_IF_ROS_CMAKE_INSTALL_PATH:="${SUMO_IF_ROS_CMAKE_BUILD_PATH}/install"
SUMO_IF_ROS_CMAKE_INSTALL_PATH:="${SUMO_IF_ROS_CMAKE_BUILD_PATH}/install"

include ${MAKE_GADGETS_PATH}/make_gadgets.mk
include ${MAKE_GADGETS_PATH}/docker/docker-tools.mk
include ${APT_CACHER_NG_DOCKER_PATH}/apt_cacher_ng_docker.mk

include ${SUMO_IF_ROS_SUBMODULES_PATH}/adore_v2x_sim/adore_v2x_sim.mk
include ${SUMO_IF_ROS_SUBMODULES_PATH}/coordinate_conversion/coordinate_conversion.mk
include ${SUMO_IF_ROS_SUBMODULES_PATH}/adore_if_ros_msg/adore_if_ros_msg.mk

include ${SUMO_IF_ROS_SUBMODULES_PATH}/cpplint_docker/cpplint_docker.mk
include ${SUMO_IF_ROS_SUBMODULES_PATH}/cppcheck_docker/cppcheck_docker.mk
include ${SUMO_IF_ROS_SUBMODULES_PATH}/lizard_docker/lizard_docker.mk

CPP_PROJECT_DIRECTORY:=${SUMO_IF_ROS_MAKEFILE_PATH}/${SUMO_IF_ROS_PROJECT}

.PHONY: build_sumo_if_ros 
build_sumo_if_ros: ## Build sumo_if_ros
	cd "${SUMO_IF_ROS_MAKEFILE_PATH}" && make

.PHONY: clean_sumo_if_ros
clean_sumo_if_ros: ## Clean sumo_if_ros build artifacts
	cd "${SUMO_IF_ROS_MAKEFILE_PATH}" && make clean

.PHONY: branch_sumo_if_ros
branch_sumo_if_ros: ## Returns the current docker safe/sanitized branch for sumo_if_ros
	@printf "%s\n" ${SUMO_IF_ROS_TAG}

.PHONY: image_sumo_if_ros
image_sumo_if_ros: ## Returns the current docker image name for sumo_if_ros
	@printf "%s\n" ${SUMO_IF_ROS_IMAGE}

endif
