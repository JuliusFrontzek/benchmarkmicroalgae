# Use the official MATLAB R2025b image (Ubuntu 24.04 based)
FROM mathworks/matlab:r2025b

USER root

# 1. Install system dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    curl \
    ca-certificates \
    build-essential \
    libpython3-dev \
    git \
    ripgrep \
    sudo \
    cmake \
    ninja-build \
    screen \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /run/screen && chmod 777 /run/screen

# 2. Install NVIDIA CUDA 12.6 toolkit for Ubuntu 24.04 (Noble)
RUN curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
    -o /tmp/cuda-keyring.deb \
    && dpkg -i /tmp/cuda-keyring.deb \
    && rm /tmp/cuda-keyring.deb \
    && apt-get update && apt-get install -y --no-install-recommends cuda-toolkit-12-6 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# 3. Grant 'matlab' user sudo privileges
RUN echo "matlab ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 4. Set MATLAB + CUDA paths
ENV MATLAB_ROOT=/opt/matlab/R2025b
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${MATLAB_ROOT}/bin:${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${MATLAB_ROOT}/bin/glnxa64:${CUDA_HOME}/lib64

# Switch to the 'matlab' user
USER matlab

# IMPORTANT: Match the VS Code default workspace path
WORKDIR /workspaces/benchmarkmicroalgae

# 5. Install 'uv' and 'Claude Code'
RUN curl -fsSL https://astral.sh/uv/install.sh | sh
ENV PATH="/home/matlab/.local/bin:${PATH}"

# 6. Create Python virtual environment in the workspace folder
RUN uv venv .venv --python 3.12

# 7. Install PyTorch with CUDA 12.4 wheels 
RUN uv pip install --extra-index-url https://download.pytorch.org/whl/cu124 torch torchvision

# 8. Install build deps + project deps
RUN uv pip install scikit-build matlabengine==25.2.* numpy pandas

# 9. Install l4casadi using the environment's torch/skbuild
RUN uv pip install --no-build-isolation l4casadi

CMD ["bash"]