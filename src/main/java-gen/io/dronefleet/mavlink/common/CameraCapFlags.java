package io.dronefleet.mavlink.common;

import io.dronefleet.mavlink.annotations.MavlinkEntryInfo;
import io.dronefleet.mavlink.annotations.MavlinkEnum;

/**
 * Camera capability flags (Bitmap) 
 */
@MavlinkEnum
public enum CameraCapFlags {
    /**
     * Camera is able to record video 
     */
    @MavlinkEntryInfo(1)
    CAMERA_CAP_FLAGS_CAPTURE_VIDEO,

    /**
     * Camera is able to capture images 
     */
    @MavlinkEntryInfo(2)
    CAMERA_CAP_FLAGS_CAPTURE_IMAGE,

    /**
     * Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE) 
     */
    @MavlinkEntryInfo(4)
    CAMERA_CAP_FLAGS_HAS_MODES,

    /**
     * Camera can capture images while in video mode 
     */
    @MavlinkEntryInfo(8)
    CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE,

    /**
     * Camera can capture videos while in Photo/Image mode 
     */
    @MavlinkEntryInfo(16)
    CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE,

    /**
     * Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE) 
     */
    @MavlinkEntryInfo(32)
    CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE,

    /**
     * Camera has basic zoom control (MAV_CMD_SET_CAMERA_ZOOM) 
     */
    @MavlinkEntryInfo(64)
    CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM,

    /**
     * Camera has basic focus control (MAV_CMD_SET_CAMERA_FOCUS) 
     */
    @MavlinkEntryInfo(128)
    CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS,

    /**
     * Camera has video streaming capabilities (request {@link io.dronefleet.mavlink.common.VideoStreamInformation VIDEO_STREAM_INFORMATION} with 
     * MAV_CMD_REQUEST_MESSAGE for video streaming info) 
     */
    @MavlinkEntryInfo(256)
    CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM,

    /**
     * Camera supports tracking of a point on the camera view. 
     */
    @MavlinkEntryInfo(512)
    CAMERA_CAP_FLAGS_HAS_TRACKING_POINT,

    /**
     * Camera supports tracking of a selection rectangle on the camera view. 
     */
    @MavlinkEntryInfo(1024)
    CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE,

    /**
     * Camera supports tracking geo status ({@link io.dronefleet.mavlink.common.CameraTrackingGeoStatus CAMERA_TRACKING_GEO_STATUS}). 
     */
    @MavlinkEntryInfo(2048)
    CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS
}
