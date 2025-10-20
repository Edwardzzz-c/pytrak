#!/bin/bash

# Build Trakstar Docker image
set -e

IMAGE_NAME=pytrak
CONTAINER_NAME=pytrak_container

echo "Building Trakstar Docker image..."
docker build -t $IMAGE_NAME .

echo "Starting container with USB privileges..."
docker run -d --name $CONTAINER_NAME \
    --net=host \
    --privileged \
    --device=/dev/bus/usb:/dev/bus/usb \
    -v $(pwd):/pytrak \
    $IMAGE_NAME \
    tail -f /dev/null

echo "Container started. To access it, run:"
echo "docker exec -it $CONTAINER_NAME /bin/bash"
echo ""
echo "Inside the container, you can run:"
echo "python3 examples/simple_example.py"
echo "python3 examples/trakstar_example.py"
