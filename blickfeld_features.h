#pragma once

// Dieser Name wird standardmäßig benutzt, wenn er nicht von einer Umgebungsvariable ersetzt wird
#define DEFAULT_NODE_NAME "blickfeld_provider"

#define SCANNER_IP "192.168.26.26"

//---------------------------------------------
// Variant configuration - please select the features you want:
#define CUSTOM_SCAN_PATTERN
#define FRAME_RATE_7
#define HOR_FOV_70
#define VER_FOV_30
#define USE_FILTERING
#define HOR_RES_MAX
#define VER_RES_NORMAL
#define INTERLEAVE_MODE
#define DISTORTION_CORRECTION

#define STREAM_IMU

#define USE_FILTERING
#define MAXIMUM_DETECT_RANGE
#define EXTENDED_SIGNAL_INTERPRET
#define STRICT_SIGNAL_EVAL
//---------------------------------------------

//---------------------------------------------
// Do not change:
#ifdef NAMED_SCAN_PATTERN
    #ifdef NAMED_SCAN_PATTERN_A
    #define NAMED_SCAN_PATTERN_ID "abc"
    #endif
    
    #ifdef NAMED_SCAN_PATTERN_B
    #define NAMED_SCAN_PATTERN_ID "def"
    #endif
    
    #ifdef NAMED_SCAN_PATTERN_C
    #define NAMED_SCAN_PATTERN_ID "ghi"
    #endif
#endif

#ifdef CUSTOM_SCAN_PATTERN
    #ifdef FRAME_RATE_3
    #define SENSOR_FRAME_RATE 3.0
    #endif

    #ifdef FRAME_RATE_5
    #define SENSOR_FRAME_RATE 5.0
    #endif

    #ifdef FRAME_RATE_7
    #define SENSOR_FRAME_RATE 7.0
    #endif

    #ifdef FRAME_RATE_10
    #define SENSOR_FRAME_RATE 10.0
    #endif

    #ifdef FRAME_RATE_12
    #define SENSOR_FRAME_RATE 12.0
    #endif

    #ifdef FRAME_RATE_15
    #define SENSOR_FRAME_RATE 15.0
    #endif

    #ifdef HOR_FOV_MAX
    #define HORIZONTAL_FOV 72.0
    #endif

    #ifdef HOR_FOV_70
    #define HORIZONTAL_FOV 70.0
    #endif

    #ifdef HOR_FOV_65
    #define HORIZONTAL_FOV 65.0
    #endif

    #ifdef HOR_FOV_60
    #define HORIZONTAL_FOV 60.0
    #endif

    #ifdef HOR_FOV_50
    #define HORIZONTAL_FOV 50.0
    #endif

    #ifdef HOR_FOV_MIN
    #define HORIZONTAL_FOV 30.0
    #endif

    #ifdef VER_FOV_MAX
    #define VERTICAL_FOV 33.0
    #endif

    #ifdef VER_FOV_30
    #define VERTICAL_FOV 30.0
    #endif

    #ifdef VER_FOV_25
    #define VERTICAL_FOV 25.0
    #endif

    #ifdef VER_FOV_20
    #define VERTICAL_FOV 10.0
    #endif

    #ifdef VER_FOV_MIN
    #define VERTICAL_FOV 10.0
    #endif

    #ifdef HOR_RES_MAX
    #define HORIZONTAL_STEP_SIZE 0.4
    #endif

    #ifdef HOR_RES_0_6
    #define HORIZONTAL_STEP_SIZE 0.6
    #endif

    #ifdef HOR_RES_0_8
    #define HORIZONTAL_STEP_SIZE 0.8
    #endif

    #ifdef HOR_RES_MIN
    #define HORIZONTAL_STEP_SIZE 1
    #endif

    #ifdef VER_RES_MIN
    #define SCANLINES_UP 24
    #define SCANLINES_DOWN 24
    #endif

    #ifdef VER_RES_NORMAL
    #define SCANLINES_UP 38
    #define SCANLINES_DOWN 38
    #endif

    #ifdef VER_RES_INCREASED
    #define SCANLINES_UP 52
    #define SCANLINES_DOWN 52
    #endif

    #ifdef VER_RES_MAX
    #define SCANLINES_UP 76
    #define SCANLINES_DOWN 76
    #endif
#endif

#ifdef USE_FILTERING
	#ifdef LIMITED_DETECT_RANGE
	#define MIN_RANGE 1.5
	#define MAX_RANGE 50
	#endif

	#ifdef MAXIMUM_DETECT_RANGE
	#define MIN_RANGE 1.5
	#define MAX_RANGE 256
	#endif

	#ifdef NORMAL_SIGNAL_INTERPRET
	#define MAX_NUMBER_OF_RETURNS 1
	#endif

	#ifdef EXTENDED_SIGNAL_INTERPRET
	#define MAX_NUMBER_OF_RETURNS 2
	#endif

	#ifdef NORMAL_SIGNAL_EVAL
	#define DELETE_NO_RETURNS false
	#endif

	#ifdef STRICT_SIGNAL_EVAL
	#define DELETE_NO_RETURNS true
	#endif
#endif
