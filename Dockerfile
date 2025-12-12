# =========================
# Builder stage
# =========================
FROM osrf/ros:jazzy-desktop-full AS builder

# -------------------------
# System dependencies
# -------------------------
RUN apt-get update && apt-get install -y \
    git vim curl wget build-essential pkg-config xdg-utils \
    libssl-dev libclang-dev libwebkit2gtk-4.1-dev \
    libayatana-appindicator3-dev librsvg2-dev \
    nodejs python3-pip ccache software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# Colcon plugins (required by rclrs build)
# -------------------------
RUN pip3 install --break-system-packages \
    git+https://github.com/colcon/colcon-cargo.git \
    git+https://github.com/colcon/colcon-ros-cargo.git

# -------------------------
# ccache
# -------------------------
ENV CCACHE_DIR=/root/.ccache
ENV PATH="/usr/lib/ccache:$PATH"
ENV CCACHE_MAXSIZE=30G

# -------------------------
# Rust
# -------------------------
ENV RUSTUP_HOME=/root/.rustup
ENV CARGO_HOME=/root/.cargo
ENV PATH="$CARGO_HOME/bin:$PATH"
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

# -------------------------
# pnpm
# -------------------------
RUN curl -fsSL https://get.pnpm.io/install.sh | bash -s -- -y
ENV PATH="/root/.local/share/pnpm:$PATH"

# -------------------------
# Copy Tauri app
# -------------------------
WORKDIR /root/
COPY ./ /root/

# -------------------------
# Build Tauri app
# -------------------------
WORKDIR /root
ENV TAURI_SKIP_APPIMAGE=true
ENV CI=true
RUN pnpm install --frozen-lockfile && pnpm tauri build

# =========================
# Runtime stage
# =========================
FROM osrf/ros:jazzy-desktop-full

# -------------------------
# Install runtime dependencies (same as builder)
# -------------------------
RUN apt-get update && apt-get install -y \
    git vim curl wget build-essential pkg-config xdg-utils \
    libssl-dev libclang-dev libwebkit2gtk-4.1-dev \
    libayatana-appindicator3-dev librsvg2-dev \
    nodejs python3-pip ccache software-properties-common fish \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# Colcon plugins (required by rclrs build)
# -------------------------
RUN pip3 install --break-system-packages \
    git+https://github.com/colcon/colcon-cargo.git \
    git+https://github.com/colcon/colcon-ros-cargo.git

# -------------------------
# ccache
# -------------------------
ENV CCACHE_DIR=/root/.ccache
ENV PATH="/usr/lib/ccache:$PATH"
ENV CCACHE_MAXSIZE=30G

# -------------------------
# Rust (system-wide installation)
# -------------------------
ENV RUSTUP_HOME=/usr/local/rustup
ENV CARGO_HOME=/usr/local/cargo
ENV PATH="$CARGO_HOME/bin:$PATH"
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path

# -------------------------
# pnpm (system-wide installation via standalone script)
# -------------------------
RUN curl -fsSL https://get.pnpm.io/install.sh | env PNPM_VERSION=10.25.0 SHELL=bash sh - && \
    mv /root/.local/share/pnpm /usr/local/lib/pnpm && \
    ln -s /usr/local/lib/pnpm/.tools/pnpm-exe/10.25.0/pnpm /usr/local/bin/pnpm && \
    ln -s /usr/local/lib/pnpm/pnpx /usr/local/bin/pnpx



# -------------------------
# Copy built Tauri app
# -------------------------
COPY --from=builder /root/src-tauri/target/release /root/src-tauri/target/release
COPY --from=builder /root/dist /root/dist

# -------------------------
# Environment setup for volume mounts
# -------------------------
ENV PATH="/usr/local/cargo/bin:$PATH"
RUN echo 'export PATH="/usr/local/cargo/bin:$PATH"' >> /etc/profile.d/dev-tools.sh
RUN echo 'export RUSTUP_HOME=/usr/local/rustup' >> /etc/profile.d/dev-tools.sh
RUN echo 'export CARGO_HOME=/usr/local/cargo' >> /etc/profile.d/dev-tools.sh
RUN echo 'export CCACHE_DIR=/root/.ccache' >> /etc/profile.d/dev-tools.sh
RUN echo 'export CCACHE_MAXSIZE=30G' >> /etc/profile.d/dev-tools.sh
RUN echo 'export PATH="/usr/lib/ccache:$PATH"' >> /etc/profile.d/dev-tools.sh

# -------------------------
# Source ROS2 and dev tools
# -------------------------
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc
RUN echo 'source /etc/profile.d/dev-tools.sh' >> /etc/bash.bashrc

WORKDIR /root/
CMD ["bash"]

