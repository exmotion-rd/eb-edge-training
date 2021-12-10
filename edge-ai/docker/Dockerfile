###########################################################################################
# Dockerfile for Jetson Nano
FROM nvcr.io/nvidia/l4t-tensorflow:r32.6.1-tf2.5-py3 as base_libs

# NVIDIAのJetson用リポジトリを追加して必要なパッケージをインストール
# 参考: https://forums.developer.nvidia.com/t/adding-https-repo-download-nvidia-com-jetson-repositories-to-docker-image/121900/15
ADD --chown=root:root https://repo.download.nvidia.com/jetson/jetson-ota-public.asc /etc/apt/trusted.gpg.d/jetson-ota-public.asc
RUN chmod 644 /etc/apt/trusted.gpg.d/jetson-ota-public.asc && \
    apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates && \
    echo "deb https://repo.download.nvidia.com/jetson/common r32.6 main" > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    echo "deb https://repo.download.nvidia.com/jetson/t210 r32.6 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list  && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        python3-pip \
        python3-matplotlib \
        libffi-dev \
        libopencv \
        libopencv-python && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 古いprotobufはアンインストール
RUN pip3 uninstall -y protobuf


###########################################################################################
# build tools
FROM base_libs AS build_tools

WORKDIR /usr/src
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        unzip

# CMake
# apt-getで入手できるものはバージョンが古くてONNX Runtimeのビルドに使用できない
WORKDIR /usr/src/cmake
ADD https://github.com/Kitware/CMake/releases/download/v3.21.4/cmake-3.21.4-linux-aarch64.tar.gz .
RUN tar xfvz cmake-3.21.4-linux-aarch64.tar.gz && \
    cd cmake-3.21.4-linux-aarch64 && \
    cp -R bin/* /usr/local/bin && \
    cp -R share/* /usr/local/share


###########################################################################################
### Protobuf(C++ implementation)
## 参考サイト: https://github.com/protocolbuffers/protobuf/blob/master/src/README.md
FROM build_tools AS protobuf_builder
WORKDIR /usr/src/protobuf

ARG PROTOBUF_VERSION=3.19.1
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        autoconf \
        libtool \
        unzip \
        python3.6-dev

ADD https://github.com/protocolbuffers/protobuf/releases/download/v${PROTOBUF_VERSION}/protobuf-python-${PROTOBUF_VERSION}.tar.gz .
RUN tar xfvz protobuf-python-${PROTOBUF_VERSION}.tar.gz
RUN pip3 install --no-cache-dir Cython setuptools wheel

RUN cd protobuf-${PROTOBUF_VERSION} && \
    ./configure --disable-shared && \
    sed -i 's/^\(C.*FLAGS\) = \(.*\)$/\1 = \2 -fPIC/' Makefile &&\
    sed -i 's/^\(C.*FLAGS\) = \(.*\)$/\1 = \2 -fPIC/' src/Makefile &&\
    make -j$(nproc) && \
    make install && \
    ldconfig

WORKDIR /usr/src/protobuf
RUN cd protobuf-${PROTOBUF_VERSION}/python && \
    python3 setup.py bdist_wheel --cpp_implementation && \
    pip3 install dist/protobuf-3.19.1-cp36-cp36m-linux_aarch64.whl

###########################################################################################
### ONNX
FROM protobuf_builder AS onnx_builder

# build tool and libs
RUN apt-get update && \
    apt install -y --no-install-recommends \
        build-essential \
        software-properties-common \
        libopenblas-dev \
        libpython3.6-dev

# ONNX Runtime (with --use_cuda, --use_tensorrt)
# 参考サイト: https://onnxruntime.ai/docs/build/eps.html#tensorrt
# 注意: CUDAなどを利用するために、docker build を nvidiaランタイムで行う必要がある
ARG ONNX_RUNTIME_VERSION=1.9.1
WORKDIR /usr/src
RUN git clone --recursive https://github.com/microsoft/onnxruntime.git -b v${ONNX_RUNTIME_VERSION} && \
    export MAKEFLAGS=-j$(nproc) && \
    export CUDACXX="/usr/local/cuda/bin/nvcc" && \
    cd onnxruntime && \
    ./build.sh --update --config Release --build --build_wheel \
                --use_cuda --cuda_home /usr/local/cuda --cudnn_home /usr/lib/aarch64-linux-gnu \
                --use_tensorrt --tensorrt_home /usr/src/tensorrt && \
    pip3 install --no-cache-dir build/Linux/Release/dist/*.whl && \
    pip3 install --no-cache-dir sympy

# ONNX
RUN pip3 install --no-cache-dir onnx

# TensorFlow-ONNX
RUN pip3 install --no-cache-dir tf2onnx

# ONNX GraphSurgeon to modify ONNX models
RUN pip3 install --no-cache-dir onnx_graphsurgeon --index-url https://pypi.ngc.nvidia.com

# ONNX Optimizer
WORKDIR /usr/src
RUN git clone --recursive https://github.com/onnx/optimizer onnxoptimizer && \
    cd onnxoptimizer && \
    python3 setup.py bdist_wheel && \
    pip3 install dist/*.whl
RUN pip3 install --no-cache-dir --no-deps onnx-simplifier

# Rmove setup tools
RUN pip3 uninstall -y Cython setuptools wheel

###########################################################################################
### JupyterLab
FROM build_tools AS jupyter_builder

# NodeJS for JupyterLab
WORKDIR /usr/src/nodejs
ARG NODEJS_VERSION=16.13.0
RUN wget https://nodejs.org/dist/v16.13.0/node-v${NODEJS_VERSION}-linux-arm64.tar.xz && \
    tar xJf node-v${NODEJS_VERSION}-linux-arm64.tar.xz && \
    cd node-v${NODEJS_VERSION}-linux-arm64 && \
    cp -R * /usr/local

# JupyterLab
RUN pip3 install --no-cache-dir packaging
RUN pip3 install --no-cache-dir \
        jupyter \
        jupyterlab && \
    jupyter labextension install @jupyter-widgets/jupyterlab-manager


###########################################################################################
FROM base_libs AS build

# copy: bin, lib
COPY --from=base_libs /usr/bin /usr/bin
COPY --from=base_libs /usr/lib /usr/lib

# copy: protobuf, onnx
COPY --from=onnx_builder /usr/local/bin /usr/local/bin/
COPY --from=onnx_builder /usr/local/lib/ /usr/local/lib/
COPY --from=onnx_builder /usr/local/include/ /usr/local/include/
COPY --from=onnx_builder /usr/local/share /usr/local/share

# copy: jupyterlab
COPY --from=jupyter_builder /usr/local/bin /usr/local/bin/
COPY --from=jupyter_builder /usr/local/lib/ /usr/local/lib/
COPY --from=jupyter_builder /usr/local/share/jupyter /usr/local/share/jupyter

# additonal packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        less \
        python3-tk \
        sudo \
        vim  \
        zlib1g-dev && \
    apt-get clean && \
    rm -r /var/lib/apt/lists/*

# update pip
RUN python3 -m pip install --upgrade pip

# additional python packages
RUN pip3 install --no-cache-dir pillow
RUN pip3 install --no-cache-dir tensorflow_datasets 
RUN pip3 install --no-cache-dir colored
RUN pip3 install --no-cache-dir psutil
RUN pip3 install --no-cache-dir tqdm

# ユーザーの追加
# ホストのディレクトリをマウントしたときに、作成するファイルの所有者をrootにしない簡易対策
#   - UIDが1000以外の人が利用することもあるケースでは不十分なため、他の対策が必要
#   - 作成するユーザを video グループに追加しているのは、GPUなどのデバイスを利用する権限を付与するため
#   - パスワード: jetson
ARG UID=1000
ARG USER=developer
ARG PASSWORD=jetson
RUN useradd -m --uid ${UID} -U --groups sudo,video ${USER} -s /bin/bash &&\
    echo ${USER}:${PASSWORD} | chpasswd

# 追加したユーザーに切り替え
USER ${USER}
ENV user=${USER}
WORKDIR /home/${USER}

# JupyterLabにリモートからアクセスするためのパスワードを設定
#   - パスワード: jetson
RUN jupyter server --generate-config
RUN python3 -c "import os; from notebook.auth.security import set_password; set_password('jetson', '/home/${user}/.jupyter/jupyter_server_config.json')"
RUN sed -i "s/#c.ServerApp.allow_origin = ''/c.ServerApp.allow_origin = '*'/" /home/${user}/.jupyter/jupyter_server_config.py
RUN sed -i "s/#c.ServerApp.ip = 'localhost'/c.ServerApp.ip = '0.0.0.0'/" /home/${user}/.jupyter/jupyter_server_config.py

# start bash
CMD /bin/bash
