FROM python:3.11-slim

WORKDIR /app

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git \
    cmake \
    build-essential \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds.git && \
    cd cyclonedds && \
    mkdir build install && \
    cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=../install && \
    cmake --build . --config RelWithDebInfo --target install && \
    cd ../.. && \
    export CYCLONEDDS_HOME="$(pwd)/cyclonedds/install"

ENV CYCLONEDDS_HOME=/app/cyclonedds/install
ENV LD_LIBRARY_PATH=${CYCLONEDDS_HOME}/lib

RUN pip install --no-cache-dir \
    git+https://github.com/eclipse-cyclonedds/cyclonedds-python \
    dearpygui \
    opencv-python \
    filterpy

# Set environment variable for display (needed for GUI apps)
ENV DISPLAY=:0

CMD ["bash"]
