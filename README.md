# Packaging System

## Prerequisites

* A host PC with Docker and Git installed.
* SocketCAN

## Installation

### 1. Clone the Repository

```bash
git clone --recurse-submodules -j8 https://github.com/samkwok-hkclr/packaging_system
```

### 2. Build the Docker image

```bash
cd packaging_system
docker build --tag packaging_system:v0 --target=dev_img --progress=plain .
```

Add the hosts when building the image if you build in mainland.

```bash
cd packaging_system
docker build --tag packaging_system:v0 \
--add-host=alive.github.com:140.82.112.25 \
--add-host=live.github.com:140.82.112.25 \
--add-host=github.githubassets.com:185.199.109.154 \
--add-host=central.github.com:140.82.114.21 \
--add-host=desktop.githubusercontent.com:185.199.111.133 \
--add-host=assets-cdn.github.com:185.199.110.153 \
--add-host=camo.githubusercontent.com:185.199.109.133 \
--add-host=github.map.fastly.net:185.199.110.133 \
--add-host=github.global.ssl.fastly.net:146.75.121.194 \
--add-host=gist.github.com:140.82.121.3 \
--add-host=github.io:185.199.108.153 \
--add-host=github.com:140.82.121.4 \
--add-host=github.blog:192.0.66.2 \
--add-host=api.github.com:140.82.121.5 \
--add-host=raw.githubusercontent.com:185.199.111.133 \
--add-host=user-images.githubusercontent.com:185.199.109.133 \
--add-host=favicons.githubusercontent.com:185.199.108.133 \
--add-host=avatars5.githubusercontent.com:185.199.110.133 \
--add-host=avatars4.githubusercontent.com:185.199.108.133 \
--add-host=avatars3.githubusercontent.com:185.199.111.133 \
--add-host=avatars2.githubusercontent.com:185.199.110.133 \
--add-host=avatars1.githubusercontent.com:185.199.108.133 \
--add-host=avatars0.githubusercontent.com:185.199.108.133 \
--add-host=avatars.githubusercontent.com:185.199.111.133 \
--add-host=codeload.github.com:140.82.121.10 \
--add-host=github-cloud.s3.amazonaws.com:3.5.12.86 \
--add-host=github-com.s3.amazonaws.com:52.217.124.121 \
--add-host=github-production-release-asset-2e65be.s3.amazonaws.com:16.15.177.240 \
--add-host=github-production-user-asset-6210df.s3.amazonaws.com:16.15.184.154 \
--add-host=github-production-repository-file-5c1aeb.s3.amazonaws.com:16.182.107.153 \
--add-host=githubstatus.com:185.199.111.153 \
--add-host=github.community:140.82.113.17 \
--add-host=github.dev:51.137.3.17 \
--add-host=collector.github.com:140.82.114.21 \
--add-host=pipelines.actions.githubusercontent.com:13.107.42.16 \
--add-host=media.githubusercontent.com:185.199.109.133 \
--add-host=cloud.githubusercontent.com:185.199.108.133 \
--add-host=objects.githubusercontent.com:185.199.109.133 \
--target=dev_img --progress=plain .
```

### 3. Run the Docker Container

```bash
docker compose up -d
```

### 4. Create and Assign Virtual CAN Tunnel to the Container

```bash
. script/activate_vxcan.bash
```
