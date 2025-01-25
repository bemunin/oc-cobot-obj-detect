#!/bin/bash
MOUNT_POINT="/mnt/ssd_nvidia"
MOUNT_SOURCE="/dev/sda1"

# Check if the mount point folder exists
if [ ! -d "$MOUNT_POINT" ]; then
    echo "Mount point $MOUNT_POINT does not exist. Existing script."
    exit 1
fi

# If check if the mount point is already mounted and contain the NVIDIA local asset route
# If not, mount the drive
if [ ! -d "$MOUNT_POINT/NVIDIA" ]; then
    echo "$MOUNT_POINT has not been mounted. Mounting device /dev/sda1."
    sudo mount $MOUNT_SOURCE $MOUNT_POINT
    echo "successfylly mount $MOUNT_SOURCE to $MOUNT_POINT assets"
fi

PARENT_DIR="$(dirname "$(realpath "$0")")"
echo "Parent directory: $PARENT_DIR"
echo $PYTHONPATH
$ISAACSIM_DIR/isaac-sim.sh --ext-folder $PARENT_DIR --enable oc.utils.cobot --enable oc.scene.cobot_simple