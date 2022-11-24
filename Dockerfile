ARG PROJECT="adore_if_v2x"

ARG ADORE_IF_ROS_MSG_TAG="latest"
ARG V2X_IF_ROS_MSG_TAG="latest"
ARG PLOTLABLIB_TAG="latest"
ARG COORDINATE_CONVERSION_TAG="latest"

FROM adore_if_ros_msg:${ADORE_IF_ROS_MSG_TAG} AS adore_if_ros_msg
FROM v2x_if_ros_msg:${V2X_IF_ROS_MSG_TAG} AS v2x_if_ros_msg
FROM plotlablib:${PLOTLABLIB_TAG} AS plotlablib 
FROM coordinate_conversion:${COORDINATE_CONVERSION_TAG} AS coordinate_conversion

FROM ros:noetic-ros-core-focal AS adore_if_v2x_requirements_base

ARG PROJECT
ARG REQUIREMENTS_FILE="requirements.${PROJECT}.ubuntu20.04.system"

RUN mkdir -p /tmp/${PROJECT}
WORKDIR /tmp/${PROJECT}
COPY files/${REQUIREMENTS_FILE} /tmp/${PROJECT}

RUN apt-get update && \
    xargs apt-get install --no-install-recommends -y < ${REQUIREMENTS_FILE} && \
    rm -rf /var/lib/apt/lists/*

COPY ${PROJECT} /tmp/${PROJECT}/${PROJECT}

ARG INSTALL_PREFIX=/tmp/${PROJECT}/${PROJECT}/build/install
RUN mkdir -p "${INSTALL_PREFIX}"

COPY --from=adore_if_ros_msg /tmp/adore_if_ros_msg /tmp/adore_if_ros_msg
WORKDIR /tmp/adore_if_ros_msg/adore_if_ros_msg/build
RUN cmake --install . --prefix ${INSTALL_PREFIX} 

COPY --from=v2x_if_ros_msg /tmp/v2x_if_ros_msg /tmp/v2x_if_ros_msg
WORKDIR /tmp/v2x_if_ros_msg/v2x_if_ros_msg/build
RUN cmake --install . --prefix ${INSTALL_PREFIX} 

COPY --from=coordinate_conversion /tmp/coordinate_conversion /tmp/coordinate_conversion
WORKDIR /tmp/coordinate_conversion/coordinate_conversion/build
RUN cmake --install . --prefix ${INSTALL_PREFIX} 

COPY --from=plotlablib /tmp/plotlablib /tmp/plotlablib
WORKDIR /tmp/plotlablib/plotlablib/build
RUN cmake --install . --prefix ${INSTALL_PREFIX} 

COPY ${PROJECT} /tmp/${PROJECT}


FROM adore_if_v2x_requirements_base AS adore_if_v2x_builder

ARG PROJECT
WORKDIR /tmp/${PROJECT}/${PROJECT}
RUN mkdir -p build 

SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/${PROJECT}/${PROJECT}/build


RUN source /opt/ros/noetic/setup.bash && \
    cmake .. && \
    cmake --build . --config Release --target install -- -j $(nproc) && \
    cpack -G DEB && find . -type f -name "*.deb" | xargs mv -t . && \
    mv CMakeCache.txt CMakeCache.txt.build

#RUN cp -r /tmp/${PROJECT}/build/devel/lib/${PROJECT} /tmp/${PROJECT}/build/install/lib/${PROJECT}

#FROM alpine:3.14

#ARG PROJECT
#COPY --from=adore_if_v2x_builder /tmp/${PROJECT}/build /tmp/${PROJECT}/build

