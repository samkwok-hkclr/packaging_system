# Packaging System

## Prerequisites

* A host PC with Docker and Git installed.
* SocketCAN

## Installation

### 1. Clone the Repository

```bash
git clone --recurse-submodules -j8 https://github.com/samkwok-hkclr/packaging_system
```

### 2. Build the Docker images

```bash
cd packaging_system
docker build --tag packaging_system:v0 --target=dev_img --progress=plain .
```

### 3. Run the Docker Container

```bash
docker compose up -d
```

### 4. Create and Assign Virtual CAN Tunnel to the Container

```bash
. script/activate_vxcan.bash
```
