# Reproducible ADAS environment: C++/pyadas build and offline scripts without host install.
#
#   tools   (default) — C++ host build, pyadas, bag analysis, visualizer, tests
#   android (--target android) — same plus JDK 17 + Android SDK/NDK for APK build
#
# Build via scripts/docker.sh or .devcontainer/docker-compose.yml.

FROM ubuntu:22.04 AS tools

# Username is fixed so paths inside the image (/home/dev) do not depend
# on the host username — only UID/GID change.
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONDONTWRITEBYTECODE=1

# DNS often returns IPv6 first, but the container has no route to it — apt and pip
# time out. Ask getaddrinfo to prefer IPv4.
RUN echo 'precedence ::ffff:0:0/96 100' >> /etc/gai.conf

RUN apt-get update && apt-get install -y --no-install-recommends \
    # base
    ca-certificates curl wget git git-lfs sudo unzip zip tar nano \
    # C++ build (conan pulls the rest)
    build-essential cmake ninja-build pkg-config \
    # protoc for generate_proto_python.sh (C++ build gets protobuf from conan)
    protobuf-compiler \
    # python and deps for matplotlib/opencv/visualizer
    python3 python3-pip python3-dev python3-tk \
    libgl1 libglib2.0-0 libsm6 libxext6 libxrender1 \
    # udev/usb — pyadas builds with libusb from conan and links
    libudev-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# User with host UID: files in the mounted repo stay owned by you.
RUN groupadd --gid $USER_GID $USERNAME 2>/dev/null || true \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME 2>/dev/null || true \
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Python deps are pinned at image build time, not on the host.
# MetaDrive is a separate file: controller run (sim.eval) will not start without it.
COPY app/src/main/scripts/requirements.txt /tmp/requirements.txt
COPY app/src/main/scripts/sim/requirements.txt /tmp/sim-requirements.txt
RUN pip3 install --no-cache-dir --upgrade pip \
    && pip3 install --no-cache-dir -r /tmp/requirements.txt \
    && pip3 install --no-cache-dir metadrive-simulator>=0.4.2.4 \
    && pip3 install --no-cache-dir "conan>=2.0,<3" \
    && rm /tmp/requirements.txt /tmp/sim-requirements.txt

# MetaDrive downloads assets into its site-packages dir and takes a file lock there.
# A normal user cannot write there, so pull assets at build time
# and open the dir for write — otherwise the run fails on assets.lock.
RUN python3 -m metadrive.pull_asset --update || python3 -m metadrive.pull_asset || true \
    && chmod -R a+rwX /usr/local/lib/python3.10/dist-packages/metadrive

USER $USERNAME
WORKDIR /workspace/adas

# Cache dirs created as the user: on first mount docker copies ownership from the image into a named
# volume, but a volume created empty is owned by root — and gradle
# then cannot write to GRADLE_USER_HOME.
RUN mkdir -p /home/dev/.gradle /home/dev/.conan2

# Conan profile is created on first run if missing (cache is usually mounted from host).
RUN conan profile detect --force >/dev/null 2>&1 || true

CMD ["/bin/bash"]


# ---------------------------------------------------------------------------
FROM tools AS android

USER root

ARG ANDROID_SDK_VERSION=11076708
ARG ANDROID_PLATFORM=android-34
ARG ANDROID_BUILD_TOOLS=34.0.0
ARG ANDROID_NDK=26.1.10909125

ENV ANDROID_HOME=/opt/android-sdk
ENV ANDROID_SDK_ROOT=$ANDROID_HOME
ENV ANDROID_NDK_ROOT=$ANDROID_HOME/ndk/$ANDROID_NDK
ENV ANDROID_NDK_HOME=$ANDROID_NDK_ROOT
ENV PATH=$PATH:$ANDROID_HOME/cmdline-tools/latest/bin:$ANDROID_HOME/platform-tools

# JDK 21: app/build.gradle requires sourceCompatibility VERSION_21; on 17 the build fails
# with "invalid source release: 21".
RUN apt-get update && apt-get install -y --no-install-recommends openjdk-21-jdk-headless \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
ENV JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64

RUN mkdir -p $ANDROID_HOME/cmdline-tools \
    && cd /tmp \
    && wget -q "https://dl.google.com/android/repository/commandlinetools-linux-${ANDROID_SDK_VERSION}_latest.zip" -O tools.zip \
    && unzip -q tools.zip -d $ANDROID_HOME/cmdline-tools \
    && mv $ANDROID_HOME/cmdline-tools/cmdline-tools $ANDROID_HOME/cmdline-tools/latest \
    && rm tools.zip \
    && yes | sdkmanager --licenses >/dev/null \
    && sdkmanager --install "platform-tools" "platforms;${ANDROID_PLATFORM}" \
                            "build-tools;${ANDROID_BUILD_TOOLS}" "ndk;${ANDROID_NDK}" >/dev/null \
    && chmod -R a+rX $ANDROID_HOME

USER $USERNAME
WORKDIR /workspace/adas
CMD ["/bin/bash"]
