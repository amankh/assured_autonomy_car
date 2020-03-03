FROM ubuntu:16.04

RUN apt-get clean
RUN apt-get -y update
RUN apt-get install -y software-properties-common wget
RUN add-apt-repository ppa:dreal/dreal -y
RUN apt-get -y update
RUN apt-get install -y bison coinor-libclp-dev g++ g++-5 flex libfl-dev libibex-dev libnlopt-dev libpython2.7-dev pkg-config python-dev zlib1g-dev


ARG BAZEL_VERSION=0.28.1
ARG BAZEL_DEBNAME=bazel_${BAZEL_VERSION}-linux-x86_64.deb
ARG BAZEL_URL=https://github.com/bazelbuild/bazel/releases/download/${BAZEL_VERSION}/${BAZEL_DEBNAME}
ARG BAZEL_SHA256=238795b1850d4c155c8f280828c344c22d096756d206b8be2ba423731dca52c0
RUN apt-get install -y --no-install-recommends wget
RUN wget ${BAZEL_URL}
RUN if echo "${BAZEL_SHA256}  ${BAZEL_DEBNAME}" | sha256sum -c; then apt-get install -y ./${BAZEL_DEBNAME} && rm ${BAZEL_DEBNAME}; else exit 1; fi

# From dreal setup
ARG DREAL_VERSION=4.19.06.3
ARG DREAL_DEBNAME=dreal_${DREAL_VERSION}_amd64.deb
ARG DREAL_URL=https://dl.bintray.com/dreal/dreal/${DREAL_DEBNAME}
ARG DREAL_SHA256=a8c7efc07de3b459eb63e9b54a475847bd5c87443e10ce87193185eb52017f6a
WORKDIR /tmp
RUN wget ${DREAL_URL}
RUN echo "${DREAL_SHA256} ${DREAL_DEBNAME}" | sha256sum -c
RUN apt-get install -y ./${DREAL_DEBNAME}
RUN apt-get install -y python3-pip
RUN pip3 install --upgrade pip
RUN pip3 install dreal
RUN pip3 install torch torchvision

# # On Ubuntu 18.04 and above, we can just install nlohmann-json-dev, but when we were trying to use
# # tensorflow//tensorflow:1.12.0-py3 container that is based on Ubuntu 16.04, we have to install manually
# RUN apt-get -y install nlohmann-json-dev
ARG JSON_VERSION=3.5.0
ARG JSON_SHA256=3564da9c5b0cf2e032f97c69baedf10ddbc98030c337d0327a215ea72259ea21
ARG JSON_URL=https://github.com/nlohmann/json/releases/download/v${JSON_VERSION}/include.zip
WORKDIR /tmp
RUN wget ${JSON_URL}
RUN echo "${JSON_SHA256} include.zip" | sha256sum -c
WORKDIR /usr
RUN unzip /tmp/include.zip && rm -f /tmp/include.zip
