# ROS2 Humble Docker Demo - Talker & Listener

A containerized ROS2 Humble demo using Python talker/listener nodes. Supports separate and unified Dockerfile approaches, with Docker Compose orchestration for host (Linux) and bridge (Windows/Mac) networking.

---

## Table of Contents

- [ROS2 Humble Docker Demo - Talker \& Listener](#ros2-humble-docker-demo---talker--listener)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Prerequisites](#prerequisites)
  - [Quick Start](#quick-start)
  - [Project Structure](#project-structure)
  - [Approach 1: Separate Dockerfiles (Recommended)](#approach-1-separate-dockerfiles-recommended)
    - [Build Images](#build-images)
    - [Run Containers](#run-containers)
  - [Approach 2: Unified Dockerfile (Reference)](#approach-2-unified-dockerfile-reference)
    - [Build Image](#build-image)
    - [Run Container](#run-container)
  - [Approach 3: Docker Compose (Recommended for Development)](#approach-3-docker-compose-recommended-for-development)
    - [Linux (Host Networking Profile)](#linux-host-networking-profile)
    - [Windows/Mac (Bridge Networking Profile)](#windowsmac-bridge-networking-profile)
  - [Verifying Communication](#verifying-communication)
  - [Troubleshooting](#troubleshooting)
    - [Listener not receiving messages](#listener-not-receiving-messages)
    - ["Cannot connect to Docker daemon"](#cannot-connect-to-docker-daemon)
    - [Bridge networking not working (Windows/Mac)](#bridge-networking-not-working-windowsmac)
    - [High CPU usage](#high-cpu-usage)
    - ["Package 'demo\_nodes\_py' not found"](#package-demo_nodes_py-not-found)
  - [Environment Variables](#environment-variables)
  - [Advanced Usage](#advanced-usage)
    - [Custom ROS\_DOMAIN\_ID](#custom-ros_domain_id)
    - [Interactive Shell](#interactive-shell)
    - [Attach to Running Container](#attach-to-running-container)
  - [Cleaning Up](#cleaning-up)
    - [Stop and Remove Containers](#stop-and-remove-containers)
    - [Remove Images](#remove-images)
    - [Remove Networks](#remove-networks)
    - [Full Cleanup (including volumes)](#full-cleanup-including-volumes)
  - [Additional Resources](#additional-resources)
  - [License](#license)
  - [Contributing](#contributing)

---

## Overview

This project demonstrates ROS2 Humble talker/listener communication using Python demo nodes in Docker containers. Three approaches are provided:

- Separate Dockerfiles for talker and listener
- Unified Dockerfile for both nodes
- Docker Compose orchestration with host/bridge networking profiles

The setup showcases ROS2 DDS communication between containers.

---

## Prerequisites

- Docker (20.10+ recommended)
- Docker Compose (v2.0+)
- Linux: Host networking support
- Windows/Mac: Docker Desktop (bridge networking)
- Basic ROS2 knowledge (nodes, topics, publishers, subscribers)

---

## Quick Start

```bash
# Linux (host networking)
docker compose --profile host up --build

# Windows/Mac (bridge networking)
docker compose --profile bridge up --build
```

---

## Project Structure

```text
.
├── Dockerfile.talker          # Dedicated talker node container
├── Dockerfile.listener        # Dedicated listener node container
├── Dockerfile.unified         # Reference: single image for both nodes
├── docker-compose.yml         # Orchestration with host/bridge profiles
├── .dockerignore              # Exclude unnecessary files from build
├── scripts/                   # Helper scripts (if any)
└── README.md                  # This file
```

---

## Approach 1: Separate Dockerfiles (Recommended)

### Build Images

```bash
docker build -f Dockerfile.talker -t ros2-talker .
docker build -f Dockerfile.listener -t ros2-listener .
```

### Run Containers

**Linux (Host Networking):**

```bash
docker run --rm --network host --name talker ros2-talker
docker run --rm --network host --name listener ros2-listener
```

**Windows/Mac (Bridge Networking):**

```bash
# Create network
docker network create ros2-net

# Run containers
docker run --rm --network ros2-net -e ROS_LOCALHOST_ONLY=0 --name talker ros2-talker
docker run --rm --network ros2-net -e ROS_LOCALHOST_ONLY=0 --name listener ros2-listener
```

---

## Approach 2: Unified Dockerfile (Reference)

### Build Image

```bash
docker build -f Dockerfile.unified -t ros2-demo:unified .
```

### Run Container

```bash
# Talker (default)
docker run --rm --network host ros2-demo:unified

# Listener (override CMD)
docker run --rm --network host ros2-demo:unified listener
```

_Advantages: single image, less maintenance. Disadvantages: less explicit, requires runtime args._

---

## Approach 3: Docker Compose (Recommended for Development)

### Linux (Host Networking Profile)

```bash
# Start services
docker compose --profile host up --build
# Stop services
docker compose --profile host down
# View logs
docker compose --profile host logs -f
```

### Windows/Mac (Bridge Networking Profile)

```bash
# Start services
docker compose --profile bridge up --build
# Stop services
docker compose --profile bridge down
# View logs
docker compose --profile bridge logs -f
```

_Profiles allow platform-specific configurations._

---

## Verifying Communication

1. **Check Logs**: Look for talker publishing and listener receiving

    ```bash
    docker compose logs -f talker
    docker compose logs -f listener
    ```

2. **Expected Output**:

   - Talker: `[INFO] [talker]: Publishing: "Hello World: X"`
   - Listener: `[INFO] [listener]: I heard: [Hello World: X]`

3. **Verify Topics**:

    ```bash
    # Linux (host profile)
    docker compose exec talker bash
    ros2 topic list
    ros2 topic echo /chatter

    # Windows/Mac (bridge profile)
    docker compose exec talker-bridge bash
    ros2 topic list
    ros2 topic echo /chatter
    ```

4. **Check Node Graph**:

  ```bash
  # Linux (host profile)
  docker compose exec talker bash
  ros2 node list
  ros2 node info /talker

  # Windows/Mac (bridge profile)
  docker compose exec talker-bridge bash
  ros2 node list
  ros2 node info /talker
  ```

---

## Troubleshooting

### Listener not receiving messages

- Ensure both containers use the same network
  - Linux: `--network host`
  - Windows/Mac: bridge network, `ROS_LOCALHOST_ONLY=0`

- Verify matching `ROS_DOMAIN_ID`

  ```bash
  # Linux (host profile)
  docker compose exec talker env | grep ROS_DOMAIN_ID
  docker compose exec listener env | grep ROS_DOMAIN_ID

  # Windows/Mac (bridge profile)
  docker compose exec talker-bridge env | grep ROS_DOMAIN_ID
  docker compose exec listener-bridge env | grep ROS_DOMAIN_ID
  ```

- Check firewall (ports 7400-7500 UDP)
  - Linux: `sudo ufw allow 7400:7500/udp`
  - Windows: Check Defender Firewall

### "Cannot connect to Docker daemon"

- Ensure Docker service is running
  - Linux: `sudo systemctl start docker`
  - Windows/Mac: Start Docker Desktop

### Bridge networking not working (Windows/Mac)

- Set `ROS_LOCALHOST_ONLY=0`
- Use same bridge network
- Consider DDS config for explicit peer discovery

### High CPU usage

- Set `ROS_DOMAIN_ID` to isolate
- Use `ROS_LOCALHOST_ONLY=1` for single host
- Configure DDS for unicast

### "Package 'demo_nodes_py' not found"

- Rebuild images:

  ```bash
  docker compose build --no-cache
  ```

---

## Environment Variables

- `ROS_DOMAIN_ID`: DDS domain for isolation (default: 0)
- `ROS_LOCALHOST_ONLY`: Restrict DDS to localhost (0=disabled, 1=enabled)
- `ROS_DISTRO`: ROS2 distribution (set to 'humble')

---

## Advanced Usage

### Custom ROS_DOMAIN_ID

```bash
# Linux/macOS
ROS_DOMAIN_ID=42 docker compose --profile <host|bridge> up --build

# Windows PowerShell
$env:ROS_DOMAIN_ID=42; docker compose --profile <host|bridge> up --build
```

### Interactive Shell

```bash
docker run -it --rm --network host ros2-talker bash
```

### Attach to Running Container

```bash
docker exec -it ros2-talker bash
```

---

## Cleaning Up

### Stop and Remove Containers

```bash
docker compose down
```

### Remove Images

```bash
docker rmi ros2-talker ros2-listener ros2-demo:unified
```

### Remove Networks

```bash
docker network rm ros2-net
```

### Full Cleanup (including volumes)

```bash
docker compose down -v
docker system prune -a
```

---

## Additional Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Docker Images](https://hub.docker.com/_/ros)
- [Docker Compose Documentation](https://docs.docker.com/compose/)
- [ROS2 DDS Configuration](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)
- [ROS2 Demo Nodes](https://github.com/ros2/demos)

---

## License

This project uses ROS2 Humble which is licensed under Apache 2.0.

## Contributing

Feel free to submit issues or pull requests for improvements.

---

**Note**: This is a demonstration project for learning ROS2 containerization. For production deployments, consider additional security hardening, resource limits, and DDS configuration tuning.
