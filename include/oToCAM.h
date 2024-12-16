// ----------------------------------------------------------------------------------------------------------
// Copyright Notice
//
// <oToCAM500-SDK> V<1.00.12>
// Copyright c2021  oToBrite Electronics Inc. All Rights Reserved.
//
// Subject to the rights and licenses expressly granted to oToBrite hereunder, no other license is granted,
// no other use is permitted, and oToBrite retains all right, title and other licensed materials and interest
// in the software, including all Copyrights and other Intellectual Property Rights.
// Any user wishing to make a commercial use of the software must contact oToBrite at sales@otobrite.com
// to arrange an appropriate license.
// The above Copyright Notice shall be included in all copies or substantial portions of the software.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
// LIMITED TO THE WARRANTIES OF ANY SAFETY, SUTABILITY, LACK OF VIRUSES, INACCURACIES, TYPOGRAPHICAL ERRORS,
// NONINFRINGEMENT. IN NO EVENT SHALL OTOBRITE BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION).
// ----------------------------------------------------------------------------------------------------------

/*! \mainpage oToCAM SDK
 *
 * \section Overview
 *
 * oToCAM SDK is a linux platform library for oToCAM depth cameras
 * The SDK allows depth and IR streaming.
 * \section Lincese
 *
 * <oToCAM500-SDK> V<1.00.12>
 * Copyright c2021  oToBrite Electronics Inc. All Rights Reserved.
 * 
 * Subject to the rights and licenses expressly granted to oToBrite hereunder, no other license is granted, 
 * no other use is permitted, and oToBrite retains all right, title and other licensed materials and interest 
 * in the software, including all Copyrights and other Intellectual Property Rights.
 * Any user wishing to make a commercial use of the software must contact oToBrite at sales@otobrite.com 
 * to arrange an appropriate license.
 * The above Copyright Notice shall be included in all copies or substantial portions of the software. 
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT 
 * LIMITED TO THE WARRANTIES OF ANY SAFETY, SUTABILITY, LACK OF VIRUSES, INACCURACIES, TYPOGRAPHICAL ERRORS, 
 * NONINFRINGEMENT. IN NO EVENT SHALL OTOBRITE BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION).
 *
 */

#ifndef __OTO_CAM_H__
#define __OTO_CAM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/*! oToCAM camera error codes */
typedef enum {
    OTOCAM_OK                               =  0,  /*!< ok */
    OTOCAM_CAMERA_INIT_FAIL                 = -1,  /*!< camera initial failed */
    OTOCAM_CAMERA_NOT_FOUND                 = -2,  /*!< camera not found */
    OTOCAM_CAMERA_ADD_FAIL                  = -3,  /*!< camera add fail */
    OTOCAM_CAMERA_CONNECT_FAIL              = -4,  /*!< camera connect fail */
    OTOCAM_CAMERA_GET_FMT_FAIL              = -5,  /*!< camera get wh fail */
    OTOCAM_CAMERA_SET_FMT_FAIL              = -6,  /*!< camera set wh fail */
    OTOCAM_CAMERA_MMAP_FAIL                 = -7,  /*!< camera MMAP fail */
    OTOCAM_CAMERA_START_FAIL                = -8,  /*!< camera start fail */
    OTOCAM_CAMERA_STOP_FAIL                 = -9,  /*!< camera stop fail */
    OTOCAM_CAMERA_GET_FRAME_FAIL            = -10, /*!< camera get frame fail */
    OTOCAM_CAMERA_GET_INFO_FAIL             = -11, /*!< camera get frame fail */
    OTOCAM_CAMERA_QUEUE_FAIL                = -12, /*!< camera queue buffer fail */
    OTOCAM_CAMERA_ENQUEUE_FAIL              = -13, /*!< camera enqueue buffer fail */
    OTOCAM_CAMERA_SET_MODE_FAIL             = -14, /*!< camera set mode fail */
    OTOCAM_CAMERA_GET_MODE_FAIL             = -15, /*!< camera get mode fail */
    OTOCAM_CAMERA_SET_EXPOSURE_TIME_FAIL    = -16, /*!< camera set exposure time fail */
    OTOCAM_CAMERA_GET_EXPOSURE_TIME_FAIL    = -17, /*!< camera get exposure time fail */
    OTOCAM_CAMERA_SET_AUTO_EXPOSURE_FAIL    = -18, /*!< camera set auto exposure fail */
    OTOCAM_CAMERA_GET_AUTO_EXPOSURE_FAIL    = -19, /*!< camera get auto exposure fail */
    OTOCAM_CAMERA_OUTPUT_POINTER_IS_NULL    = -20, /*!< camera get auto exposure fail */
    OTOCAM_CAMERA_AE_WIDTH_TOO_BIG          = -21, /*!< camera get auto exposure value limit, width out of range */
    OTOCAM_CAMERA_AE_HEIGHT_TOO_BIG         = -22, /*!< camera get auto exposure value limit, height out of range */
    OTOCAM_CAMERA_OPEN_I2C_FAIL             = -23, /*!< camera open i2c channel fail */
    OTOCAM_CAMERA_WRITE_I2C_FAIL            = -24, /*!< camera write data to i2c register fail */
    OTOCAM_CAMERA_READ_I2C_FAIL             = -25, /*!< camera read data to i2c register fail */
    OTOCAM_CAMERA_NOT_STREAMING_ON          = -26, /*!< camera is not streaming on status */
    OTOCAM_CAMERA_DEFAULT_EXPOSURE_ERROR    = -27, /*!< camera default exposure time is out of range, the minimum value is 6, the maximum value is 1000 */
    OTOCAM_CAMERA_OVERTEMPERATURE           = -28, /*!< camera temperature is exceed threshold */
    OTOCAM_CAMERA_THREAD_TIMEOUT            = -29, /*!< background thread exceed 30 seconds to feed watch dog. If enable streaming, and exceed 10 second doesn't get image, the back ground thread will stop and force stop streaming*/
    OTOCAM_CAMERA_LASER_FAIL                = -30, /*!< laser driver detect error, force stop streaming */
    OTOCAM_CAMERA_OUTPUT_TYPE_ERROR         = -31, /*!< the output frame type selection is not in the correct type range */
    OTOCAM_CAMERA_MALLOC_FAIL               = -32, /*!< malloc buffer fail */
    OTOCAM_SDK_INIT_FAIL                    = -33, /*!< SDK init fail */
} otocam_camera_error_t;

/*! Status codes */
typedef enum {
    OTOCAM_STATUS_STREAM_ON         	=  2,  /*!< status camera stream on */
    OTOCAM_STATUS_CONNECTED		=  1,  /*!< status camera connected */
    OTOCAM_STATUS_SDK_INIT_DONE		=  0,  /*!< status sdk is init done, if need any operation, please connect camera first */
    OTOCAM_STATUS_LOST              	= -1,  /*!< status camera lost */
    OTOCAM_STATUS_NOT_CAMERA        	= -2,  /*!< status camera id is not in the camera list */
} otocam_status_t;

/*! oToCAM Device mode */
typedef enum {
    OTOCAM_MODE_NEAR                =  0, /*!< near range mode */
    OTOCAM_MODE_FAR                 =  1, /*!< far range mode */
} otocam_distance_mode_t;

/*! oToCAM Device Scenario */
typedef enum {
    OTOCAM_SCENARIO_INDOOR          =  1, /*!< near&far range indoor mode */
    OTOCAM_SCENARIO_OUTDOOR         =  2, /*!< far range outdoor mode */
} otocam_scenario_t;

typedef enum {
    OTOCAM_AE_ON          =  1, /*!< auto exposure on */
    OTOCAM_AE_OFF         =  0, /*!< auto exposure off */
} otocam_auto_exposure_t;

/*! oToCAM frame mode */
typedef enum {
    OTOCAM_FRAME_DATA_IR                =  0, /*!< frame ir data */
    OTOCAM_FRAME_DATA_DEPTH             =  1, /*!< frame depth data */
    OTOCAM_FRAME_DATA_PCL               =  2, /*!< frame pcl data */
    OTOCAM_FRAME_DATA_RAW		=  3, /*!< frame raw data */
} otocam_frame_type_mask_t;

/*! oToCAM frame header structure */
typedef struct otocam_frame_t{
    uint16_t width;             /*!< the width of the image */
    uint16_t height;            /*!< the height of the image */
    uint8_t *data;              /*!< the pointer to the first pixel data. */
    uint32_t length;            /*!< data size in bytes */
    long tv_sec;                /*!< second of timestamp */
    long tv_usec;               /*!< microsecond of timestamp */
}otocam_frame_t;

/*! oToCAM debug register inforamtion structure */
typedef struct otocam_debug_register_t{
    uint32_t xvs_count;
    uint32_t fs_count;
    uint32_t trans_frame_count;
    uint32_t ebd_err_count;
    uint32_t mipi_rx_err_count;
    uint32_t sdsw_size_err_count;

    uint32_t sdsw_ovf_err_count;
    uint32_t sdsw_conf_err_count;
    uint32_t itofisp_err_count;
    uint32_t imc_err_count;
    uint32_t hw_err_reason;
    uint32_t drop_invalid_frame_disable;
}otocam_debug_register_t, *otocam_debug_register_tp;

/*! oToCAM camera laser driver temperature structure */
typedef struct otocam_ld_temperature_t{
    float temperature_ch1;
    float temperature_ch2;
    float temperature_ch3;
    float temperature_ch4;
}otocam_ld_temperature_t, *otocam_ld_temperature_tp;

/*! oToCAM auto exposure structure */
typedef struct otocam_exposure_t{
    uint32_t open_type;         /*!< Set auto exposure on/off ::otocam_auto_exposure_t*/

    uint32_t x;                 /*!< ROI start x */
    uint32_t y;                 /*!< ROI start y */
    uint32_t width;             /*!< ROI width */
    uint32_t height;            /*!< ROI height */
    uint32_t time_range_min;    /*!< Exposure time min range */
    uint32_t time_range_max;    /*!< Exposure time max range */
    uint32_t active_typical;    /*!< Active typical exposure time */
}otocam_exposure_t;

/**
* @brief Initialize SDK all buffers and find all camera,
* if plugging or unplugging camera, should initial again.
* @brief If had opened camera, this initial function will close that one.
* @return Camera error, type of ::otocam_camera_error_t
*/
int otocam_sdk_initialize();

/**
* @brief Connect camera by camera id, please call ::otocam_get_camera_count and ::otocam_get_camera_id first to get all camera id.
* @param [in] camera_id : it can get from ::otocam_get_camera_id
* @return Camera error, type of ::otocam_camera_error_t
*/
int otocam_connect_camera( int camera_id );

/**
* @brief Camera stream on, camera start output frame,
* before get image streaming, please call ::otocam_connect_camera to connect camera.
* @param [in] camera_id : input selected camera id
* @return Camera error, type of ::otocam_camera_error_t
*/
int otocam_start_stream( int camera_id );

/**
* @brief Camera stream off, camera stop output frame.
* @param [in] camera_id : input selected camera id
* @return Camera error, type of ::otocam_camera_error_t
*/
int otocam_stop_stream( int camera_id );

/**
* @brief Catch one frame from camera, if all data is 0, or data pointer is null, please call ::otocam_get_camera_last_error to get error reponse.
* @param [in] camera_id : input selected camera id
* @return Frame data and information, type of ::otocam_frame_t
*/
otocam_frame_t otocam_wait_new_frame( int camera_id );

/**
* @brief Extract frame by need type.
* @param [in] frame : input raw frame
* @param [in] frame_type : type of ::otocam_frame_type_mask_t
* @param [out] error : camera error, type of  ::otocam_camera_error_t
* @return Frame data and information, type of ::otocam_frame_t
*/
otocam_frame_t otocam_extract_frame( otocam_frame_t frame, int frame_type, int* error );

/**
* @brief Get frame resolution information.
* @param [in] camera_id : input selected camera id
* @param [out] width : it will output frame width
* @param [out] height : it will output frame height
* @return Camera error, type of ::otocam_camera_error_t
*/
int otocam_get_camera_frame_size( int camera_id, uint16_t *width, uint16_t *height );

/**
* @brief Set frame resolution information.
* This function must be used before stream start.
* @param [in] camera_id : input selected camera id
* @param [in] width : input frame width
* @param [in] height : input frame height
* @return Camera error, type of ::otocam_camera_error_t
*/
int otocam_set_camera_frame_size( int camera_id, uint16_t width, uint16_t height );

/**
* @brief Switch the near mode or far mode when camera streaming.
* If set OTOCAM_MODE_NEAR, scenario auto changed to OTOCAM_SCENARIO_INDOOR mode.
* @param [in] camera_id : input selected camera id
* @param [in] mode : type of ::otocam_distance_mode_t
* @return Camera error, type of ::otocam_camera_error_t
*/
int otocam_switch_camera_distance_mode( int camera_id, int mode );

/**
* @brief Switch the camera indoor or outdoor mode when camera streaming,
* If set OTOCAM_SCENARIO_OUTDOOR, distance_mode auto changed to OTOCAM_MODE_FAR mode.
* @param [in] camera_id : input selected camera id
* @param [in] scenario : type of ::otocam_scenario_t
* @return Camera error, type of ::otocam_camera_error_t
*/
int otocam_switch_camera_scenario( int camera_id, int scenario );

/**
* @brief Set the auto exposure on/off when camera connected.
* @param [in] camera_id : input selected camera id
* @param [in] exposure_data : Auto exposure setting data, type of ::otocam_exposure_t
* @return Camera error, type of ::otocam_camera_error_t
*/
int otocam_set_auto_exposure( int camera_id, otocam_exposure_t exposure_data );

/**
* @brief Get the auto exposure current settings.
* @param [in] camera_id : input selected camera id
* @param [out] exposure_data : Auto exposure setting data, type of ::otocam_exposure_t
* @return Camera error, type of ::otocam_camera_error_t
*/
int otocam_get_auto_exposure( int camera_id, otocam_exposure_t* exposure_data );

/**
* @brief Set the default exposure time value, if call ::otocam_reset_camera_exposure_time function,
* the camera will be setting exposure time as this value.
* The exposesure time range is 6~1000us.
* If doesn't call this function, the default value is 250us.
* @param [in] time : input the default value
* @return Camera error, type of ::otocam_camera_error_t
*/
int otocma_set_default_exposure_time_value( int time );

/**
 * @brief Reset the exposure time to default value,
 * the default value will be 250us unless call ::otocma_set_default_exposure_time to change it.
 * @param [in] camera_id : input selected camera id
 * @return Camera error, type of ::otocam_camera_error_t
 */
int otocam_reset_camera_exposure_time( int camera_id );

/**
* @brief Set camera exposure time, the value range is 6~1000us.
* @param [in] camera_id : input selected camera id
* @param [in] time : need exposure time
* @return Camera error, type of ::otocam_camera_error_t
*/
int otocam_set_camera_exposure_time( int camera_id, int time );

/**
 * @brief Get camera current exposure time value.
 * @param [in] camera_id : input selected camera id
 * @param [out] time : current exposure time
 * @return Camera error, type of ::otocam_camera_error_t
 */
int otocam_get_camera_exposure_time(int camera_id, int* time );

/**
 * @brief Get camera current distance mode and scenario.
 * @param [in] camera_id : input selected camera id
 * @param [out] mode : current camera mode, type of ::otocam_distance_mode_t and ::otocam_scenario_t
 * @return Camera error, type of ::otocam_camera_error_t
 */
int otocam_get_camera_mode( int camera_id, int* mode );

/**
* @brief Get a count of found camera,
* please call ::otocam_sdk_initialize before get camera count.
* @note : In this API, it will list otocam only.
* @return Camera count
*/
int otocam_get_camera_count();

/**
* @brief Get camera id of found camera,
* please call ::otocam_get_camera_count before get camera id.
* @param [in] index : 0~Count-1 ::otocam_get_camera_count
* @return Camera id
*/
int otocam_get_camera_id( int index );

/**
* @brief Get camera name of found camera,
* please call ::otocam_get_camera_count before get camera name.
* @param [in] index : 0~Count-1 ::otocam_get_camera_count
* @return Camera name
*/
char* otocam_get_camera_name( int index );

/**
* @brief Get driver name of found camera,
* please call ::otocam_get_camera_count before get camera driver.
* @param [in] index : 0~Count-1 ::otocam_get_camera_count
* @return Camera driver name
*/
char* otocam_get_camera_driver_name( int index );

/**
* @brief Get camera status, the status value please reference ::otocam_status_t.
* @param [in] camera_id : input selected camera id
* @return Camera status, type of ::otocam_status_t
*/
int otocam_get_camera_status( int camera_id );

/**
* @brief Get camera last error, the error value please reference ::otocam_camera_error_t.
* @param [in] camera_id : input selected camera id
* @return Camera error, type of ::otocam_camera_error_t
*/
int otocam_get_camera_last_error( int camera_id );

/**
* @brief Get camera laser error message.
* @brief After OTOCAM_CAMERA_LASER_FAIL error, call this function to get detail message.
* @param [in] camera_id : input selected camera id
* @return Laser Error Message
*/
char* otocam_get_camera_laser_error_msg( int camera_id );

/**
* @brief Get camera EEPROM error message when camera connected.
* @brief It can check the camera EEPROM is right.
* @param [in] camera_id : input selected camera id
* @return EEPROM Error Message
*/
char* otocam_get_camera_eeprom_error_msg( int camera_id );

/**
* @brief Get all camera laser driver temperature, the unit is degree Cesius.
* Please call this function more than one second interval.
* If the temperature data pointer is NULL,
* please call ::otocam_get_camera_last_error to get error reponse.
* @param [in] camera_id : input selected camera id
* @return All camera laser driver temperature or null pointer
*/
otocam_ld_temperature_tp otocam_get_camera_temperature( int camera_id );

/**
* @brief Get debug register inforamtion of CXD5639, if get NULL pointer, please call ::otocam_get_camera_last_error to get error reponse.
* @param [in] camera_id : input selected camera id
* @return Camera debug register value or null pointer
*/
otocam_debug_register_tp otocam_get_camera_debug_register_value( int camera_id );

/**
* @brief Disconnect camera by camear id.
* @param [in] camera_id : input selected camera id
* @return None
*/
void otocam_disconnect_camera( int camera_id );

/**
* @brief Disconnect all camera release all memory,
* if wanted re-connect camera, please call ::otocam_sdk_initialize first.
* @return None
*/
void otocam_disconnect_all_camera();

/**
* @brief Get version of SDK.
* @return Version string
*/
const char* otocam_get_sdk_version();

#ifdef __cplusplus
}
#endif

#endif//__OTO_cAM_H__
// Ref : \librealsense-master\examples\C\color\rs-color.c
