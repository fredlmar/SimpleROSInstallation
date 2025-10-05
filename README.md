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
- [CustomNode - Talker Monitoring](#customnode---talker-monitoring)
  - [Overview](#overview-1)
    - [Functionality](#functionality)
    - [Topics](#topics)
    - [Parameters](#parameters)
  - [Development Workflow (Volume Mounts)](#development-workflow-volume-mounts)
  - [Testing CustomNode](#testing-customnode)
    - [Test 1: Normal Operation](#test-1-normal-operation)
    - [Test 2: Talker Timeout Detection](#test-2-talker-timeout-detection)
    - [Test 3: Talker Recovery](#test-3-talker-recovery)
    - [Test 4: Custom Parameters](#test-4-custom-parameters)
  - [Approach 1: Separate Dockerfiles (Recommended)](#approach-1-separate-dockerfiles-recommended)
    - [Build Images](#build-images)
    - [Run Containers](#run-containers)
  - [Approach 2: Unified Dockerfile (Reference)](#approach-2-unified-dockerfile-reference)
    - [Build Image](#build-image)
    - [Run Container](#run-container)
  - [Approach 3: Docker Compose (Recommended for Development)](#approach-3-docker-compose-recommended-for-development)
    - [Linux (Host Networking Profile)](#linux-host-networking-profile)
    - [Windows/Mac (Bridge Networking Profile)](#windowsmac-bridge-networking-profile)
    - [Rebuilding CustomNode Workspace](#rebuilding-customnode-workspace)
    - [Editing CustomNode Code](#editing-customnode-code)
  - [Verifying Communication](#verifying-communication)
    - [CustomNode not detecting talker status](#customnode-not-detecting-talker-status)
    - [Volume mount issues (Windows)](#volume-mount-issues-windows)
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

**CustomNode:** In addition to the talker and listener, a third component called CustomNode is included. CustomNode monitors the `/chatter` topic and publishes status to `/talker_status`, detecting when the talker stops publishing.

The setup showcases ROS2 DDS communication between containers, including monitoring and status reporting via CustomNode.

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
├── Dockerfile.custom          # CustomNode development container
├── custom_entrypoint.sh       # Entrypoint for building workspace
├── docker-compose.yml         # Orchestration with host/bridge profiles
├── .dockerignore              # Exclude unnecessary files from build
├── scripts/                   # Helper scripts (if any)
├── ros2_ws/                   # Local ROS2 workspace (volume-mounted)
│   └── src/
│       └── custom_nodes/      # CustomNode package
│           ├── package.xml
│           ├── setup.py
│           ├── resource/
│           └── custom_nodes/
│               ├── __init__.py
│               └── custom_node.py
└── README.md                  # This file
```

# CustomNode - Talker Monitoring

## Overview

CustomNode is a monitoring node that subscribes to the `/chatter` topic (published by the talker) and publishes status updates to `/talker_status`. It detects when the talker stops publishing and reports status changes.

### Functionality

- Subscribes to `/chatter` topic (`std_msgs/String`)
- Monitors message reception with a configurable timeout (default: 3 seconds)
- Publishes `talker_stopped` to `/talker_status` when timeout occurs
- Publishes `talker_active` when talker resumes

### Topics

- **Input:** `/chatter` (`std_msgs/String`) - monitors talker messages
- **Output:** `/talker_status` (`std_msgs/String`) - publishes "talker_active" or "talker_stopped"

### Parameters

- `timeout_seconds` (default: 3.0) - time before declaring talker stopped
- `check_period_seconds` (default: 1.0) - how often to check for timeout

## Development Workflow (Volume Mounts)

The `ros2_ws/` directory is volume-mounted to `/ros2_ws` in the container, enabling local development:

1. Edit Python files locally in `ros2_ws/src/custom_nodes/custom_nodes/custom_node.py`
2. The container automatically rebuilds the workspace on startup via `custom_entrypoint.sh`
3. Restart the container to apply changes: `docker compose restart custom-node`

**Benefits:** No need to rebuild the Docker image for code changes. Changes to `package.xml` or `setup.py` require container restart.

## Testing CustomNode

### Test 1: Normal Operation

- Start all services
- Verify CustomNode receives messages from talker
- Check that no timeout warnings appear

### Test 2: Talker Timeout Detection

- Stop talker container: `docker compose stop talker`
- Wait 3+ seconds
- Check CustomNode logs for "Talker stopped" warning
- Verify `/talker_status` publishes "talker_stopped"

### Test 3: Talker Recovery

- Restart talker: `docker compose start talker`
- Check CustomNode logs for "Talker resumed" message
- Verify `/talker_status` publishes "talker_active"

### Test 4: Custom Parameters

- Run with custom timeout: `docker compose run custom-node ros2 run custom_nodes custom_node --ros-args -p timeout_seconds:=5.0`

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

> **Note:** The `custom-node` service is included by default and mounts `./ros2_ws:/ros2_ws` for live code editing. View logs with `docker compose logs -f custom-node`.

**Service names:**

- `talker`, `listener`, `custom-node`

**Example commands:**

```bash
# View logs
docker compose --profile host logs -f talker
docker compose --profile host logs -f listener
docker compose --profile host logs -f custom-node

# Exec into container
docker compose --profile host exec custom-node bash

# Echo /talker_status topic
docker compose --profile host exec custom-node ros2 topic echo /talker_status
```

> **Note:** The `custom-node` service is included by default and mounts `./ros2_ws:/ros2_ws` for live code editing. View logs with `docker compose logs -f custom-node`.

```bash
# Start services
docker compose --profile host up --build
# Stop services
docker compose --profile host down
# View logs
docker compose --profile host logs -f
```

### Windows/Mac (Bridge Networking Profile)

> **Note:** The `custom-node-bridge` service is included by default and mounts `./ros2_ws:/ros2_ws` for live code editing. View logs with `docker compose logs -f custom-node-bridge`.

**Service names:**

- `talker-bridge`, `listener-bridge`, `custom-node-bridge`

**Example commands:**

```bash
# View logs
docker compose --profile bridge logs -f talker-bridge
docker compose --profile bridge logs -f listener-bridge
docker compose --profile bridge logs -f custom-node-bridge

# Exec into container
docker compose --profile bridge exec custom-node-bridge bash

# Echo /talker_status topic
docker compose --profile bridge exec custom-node-bridge ros2 topic echo /talker_status
```

> **Note:** The `custom-node-bridge` service is included by default and mounts `./ros2_ws:/ros2_ws` for live code editing. View logs with `docker compose logs -f custom-node-bridge`.

### Rebuilding CustomNode Workspace

To rebuild the CustomNode workspace after code changes:

```bash
# Restart the container to trigger an automatic rebuild
docker compose restart custom-node

# Or rebuild manually inside the container
docker compose exec custom-node bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run custom_nodes custom_node
```

### Editing CustomNode Code

- Edit Python files in `ros2_ws/src/custom_nodes/custom_nodes/custom_node.py`
- Restart the container: `docker compose restart custom-node`
- View logs: `docker compose logs -f custom-node`

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

1. **Check Logs**: Look for talker publishing, listener receiving, and CustomNode monitoring

  ```bash
  # Host profile (Linux)
  docker compose --profile host logs -f talker
  docker compose --profile host logs -f listener
  docker compose --profile host logs -f custom-node

  # Bridge profile (Windows/Mac)
  docker compose --profile bridge logs -f talker-bridge
  docker compose --profile bridge logs -f listener-bridge
  docker compose --profile bridge logs -f custom-node-bridge
  ```

2. **Expected Output**:

   - Talker: `[INFO] [talker]: Publishing: "Hello World: X"`
   - Listener: `[INFO] [listener]: I heard: [Hello World: X]`
   - CustomNode: `[INFO] [custom_node]: CustomNode started, monitoring "chatter" topic.`
   - CustomNode: `[WARN] [custom_node]: Talker stopped - no messages for 3.XX seconds`

3. **Verify Topics**:

  ```bash
  # Host profile (Linux)
  docker compose --profile host exec talker bash
  ros2 topic list
  ros2 topic echo /chatter
  docker compose --profile host exec custom-node bash
  ros2 topic echo /talker_status

  # Bridge profile (Windows/Mac)
  docker compose --profile bridge exec talker-bridge bash
  ros2 topic list
  ros2 topic echo /chatter
  docker compose --profile bridge exec custom-node-bridge bash
  ros2 topic echo /talker_status
  ```

4. **Check Node Graph**:

  ```bash
  # Host profile (Linux)
  docker compose --profile host exec talker bash
  ros2 node list
  ros2 node info /talker
  docker compose --profile host exec custom-node bash
  ros2 node info /custom_node

  # Bridge profile (Windows/Mac)
  docker compose --profile bridge exec talker-bridge bash
  ros2 node list
  ros2 node info /talker
  docker compose --profile bridge exec custom-node-bridge bash
  ros2 node info /custom_node
  ```

### CustomNode not detecting talker status

- Check that CustomNode is subscribed to `/chatter`:

  ```bash
  # Host profile
  docker compose --profile host exec custom-node ros2 topic info /chatter
  # Bridge profile
  docker compose --profile bridge exec custom-node-bridge ros2 topic info /chatter
  ```

- Verify timeout parameter:

  ```bash
  # Host profile
  docker compose --profile host exec custom-node ros2 param get /custom_node timeout_seconds
  # Bridge profile
  docker compose --profile bridge exec custom-node-bridge ros2 param get /custom_node timeout_seconds
  ```

- Check workspace build:

  ```bash
  # Host profile
  docker compose --profile host logs custom-node | grep colcon
  # Bridge profile
  docker compose --profile bridge logs custom-node-bridge | grep colcon
  ```

- Rebuild workspace:

  ```bash
  # Host profile
  docker compose --profile host restart custom-node
  # Bridge profile
  docker compose --profile bridge restart custom-node-bridge
  ```

### Volume mount issues (Windows)

- Ensure Docker Desktop has file sharing enabled for the project directory
- Check volume mount:

  ```bash
  docker compose --profile bridge exec custom-node-bridge ls -la /ros2_ws/src
  ```

- Verify permissions: files should be readable by dev user (UID 1000)

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
