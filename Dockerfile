ARG PROJECT="adore_if_v2x"
ARG REQUIREMENTS_FILE="requirements.${PROJECT}.build.ubuntu20.04.system"

FROM v2x_if_ros_msg:latest AS v2x_if_ros_msg
FROM adore_if_ros_msg:latest AS adore_if_ros_msg
FROM coordinate_conversion:latest AS coordinate_conversion
FROM plotlablib:latest AS plotlablib

FROM ros:noetic-ros-core-focal AS adore_if_v2x_builder


ARG PROJECT
ARG REQUIREMENTS_FILE


RUN mkdir -p /tmp/${PROJECT}
WORKDIR /tmp/${PROJECT}
copy files/${REQUIREMENTS_FILE} /tmp/${PROJECT}


RUN apt-get update && \
    xargs apt-get install --no-install-recommends -y < ${REQUIREMENTS_FILE} && \
    rm -rf /var/lib/apt/lists/*

COPY --from=v2x_if_ros_msg /tmp/v2x_if_ros_msg /tmp/v2x_if_ros_msg
WORKDIR /tmp/v2x_if_ros_msg/v2x_if_ros_msg/build
RUN cmake --install . --prefix /tmp/${PROJECT}/build/install

COPY --from=adore_if_ros_msg /tmp/adore_if_ros_msg /tmp/adore_if_ros_msg
WORKDIR /tmp/adore_if_ros_msg/build
RUN cmake --install . --prefix /tmp/${PROJECT}/build/install

COPY --from=coordinate_conversion /tmp/coordinate_conversion /tmp/coordinate_conversion
WORKDIR /tmp/coordinate_conversion/build
RUN cmake --install . --prefix /tmp/${PROJECT}/build/install

COPY --from=plotlablib /tmp/plotlablib /tmp/plotlablib
WORKDIR /tmp/plotlablib/plotlablib/build
RUN cmake --install . --prefix /tmp/${PROJECT}/build/install


COPY ${PROJECT} /tmp/${PROJECT}
#copy files/catkin_build.sh /tmp/${PROJECT}

WORKDIR /tmp/${PROJECT}
RUN mkdir -p build 
SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/${PROJECT}/build


RUN source /opt/ros/noetic/setup.bash && \
    cmake .. && \
    cmake --build . --config Release --target install -- -j $(nproc) && \
    cpack -G DEB && find . -type f -name "*.deb" | xargs mv -t . 

RUN cp -r /tmp/${PROJECT}/build/devel/lib/${PROJECT} /tmp/${PROJECT}/build/install/lib/${PROJECT}

FROM alpine:3.14

ARG PROJECT
COPY --from=adore_if_v2x_builder /tmp/${PROJECT}/build /tmp/${PROJECT}/build

