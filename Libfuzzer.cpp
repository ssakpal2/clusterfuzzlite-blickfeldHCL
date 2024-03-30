#include <cstdint>
#include <cstddef>
#include <iostream>
#include <cassert>
#include <cstring> // Include for std::memcpy
#include <memory> // Include for smart pointers

// Include the header file containing the classes and methods you want to fuzz
#include "blickfeld_functions.hpp"
#include "blickfeld_features.hpp" // Include the feature program

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) { 
    try {
        // Fuzzing ScanPattern
        blickfeld_functions::ScanPattern scan_pattern;
        if (size >= sizeof(blickfeld::protocol::config::ScanPattern)) {
            blickfeld::protocol::config::ScanPattern fuzzed_scan_pattern;
            memcpy(&fuzzed_scan_pattern, data, sizeof(blickfeld::protocol::config::ScanPattern));
            scan_pattern.set_pattern(fuzzed_scan_pattern);
        }
        
        // Fuzzing named scan pattern
        #ifdef NAMED_SCAN_PATTERN
        if (size > 0) {
            std::string named_scan_pattern(reinterpret_cast<const char*>(data), size);
            blickfeld_functions::set_scan_pattern(named_scan_pattern);
        }
        #endif
        
        // Prepare point cloud stream
       // std::unique_ptr<blickfeld_functions::BlickfeldLiDAR> lidar = std::make_unique<blickfeld_functions::BlickfeldLiDAR>();
       // if (lidar && lidar->is_running()) {
       //     #ifdef USE_FILTERING
        //    std::unique_ptr<blickfeld_functions::PointCloudFilter> point_cloud_filter = std::make_unique<blickfeld_functions::PointCloudFilter>();
        //    lidar->prepare_point_cloud_stream(point_cloud_filter.get());
        //    #else
         //   lidar->prepare_point_cloud_stream();
         //   #endif
        //} else {
          //  std::cerr << "Error: Lidar is not running." << std::endl;
          //  return 1; // Return non-zero value to indicate failure
       // }
        
        // Fuzzing IMU stream
       // #ifdef STREAM_IMU
       // if (lidar && lidar->is_running()) {
       //     blickfeld_functions::BlickfeldIMU imu(lidar.get());
       // } else {
       //     std::cerr << "Error: Lidar is not running." << std::endl;
       //     return 1; // Return non-zero value to indicate failure
       // }
       // #endif
    } catch (const std::exception& e) {
        // Handle any exceptions
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1; // Return non-zero value to indicate failure
    }
    
    return 0;
}
