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
# Build Tauri + rclrs
# -------------------------
ENV TAURI_SKIP_APPIMAGE=true


ENV CI=true
RUN pnpm install --frozen-lockfile && pnpm tauri build

# =========================
# Runtime stage
# =========================
FROM osrf/ros:jazzy-desktop-full

# -------------------------
# Install runtime dependencies
# -------------------------
RUN apt-get update && apt-get install -y \
    curl nodejs \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# Install pnpm
# -------------------------
RUN curl -fsSL https://get.pnpm.io/install.sh | bash -s -- -y
ENV PATH="/root/.local/share/pnpm:$PATH"

# -------------------------
# Copy built Tauri app
# -------------------------
COPY --from=builder /root/src-tauri/target/release /root/src-tauri/target/release
COPY --from=builder /root/dist /root/dist

# -------------------------
# Environment
# -------------------------
ENV PATH="/root/.cargo/bin:/root/.local/share/pnpm:$PATH"
ENV CCACHE_DIR=/root/.ccache

# -------------------------
# Source ROS2
# -------------------------
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

WORKDIR /root/src-tauri
CMD ["bash"]

