#!/bin/bash

VENDOR_PRODUCT_ID_1="045e:0779"  # HD 3000 Webcam
VENDOR_PRODUCT_ID_2="046d:0843"  # C930 Webcam

find_video_device() {
    local vendor_product="$1"

    # Find the correct video device using udevadm
    for video in /dev/video*; do
        [[ -e "$video" ]] || continue  # Skip non-existent devices

        # Query udevadm for device properties
        DEVICE_INFO=$(udevadm info --query=all --name="$video")

        # Check if the vendor:product ID matches
        if echo "$DEVICE_INFO" | grep -q "ID_VENDOR_ID=$(echo "$vendor_product" | cut -d: -f1)" && \
           echo "$DEVICE_INFO" | grep -q "ID_MODEL_ID=$(echo "$vendor_product" | cut -d: -f2)"; then
            echo "$video"
            return 0
        fi
    done

    echo "No video device found for vendor:product ID $vendor_product" >&2
    return 1
}

# Find video devices for both webcams and check the return code
VIDEO_DEVICE_1=$(find_video_device "$VENDOR_PRODUCT_ID_1")
if [[ $? -eq 0 ]]; then
    export WEBCAM_1="$VIDEO_DEVICE_1"
    echo "WEBCAM_1=$WEBCAM_1"
fi

VIDEO_DEVICE_2=$(find_video_device "$VENDOR_PRODUCT_ID_2")
if [[ $? -eq 0 ]]; then
    export WEBCAM_2="$VIDEO_DEVICE_2"
    echo "WEBCAM_2=$WEBCAM_2"
fi
