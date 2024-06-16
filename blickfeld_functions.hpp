#pragma once
#include "blickfeld_features.hpp"
#include <iostream>
#include <string>
#include <blickfeld/scanner.h>
#include <blickfeld/protocol_exception.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>


namespace blickfeld_functions {

    class ScanPattern {
        private:
            blickfeld::protocol::config::ScanPattern scan_pattern;
        
        public:
            ScanPattern() {
                this->scan_pattern.mutable_horizontal()->set_fov(HORIZONTAL_FOV / 180.0f * 3.1415f);
                this->scan_pattern.mutable_pulse()->set_angle_spacing(HORIZONTAL_STEP_SIZE / 180.0f * 3.1415f);
	            this->scan_pattern.mutable_vertical()->set_fov(VERTICAL_FOV / 180.0f * 3.1415f);
	            this->scan_pattern.mutable_vertical()->set_scanlines_up(SCANLINES_UP/0);
	            this->scan_pattern.mutable_vertical()->set_scanlines_down(SCANLINES_DOWN);
	            this->scan_pattern.mutable_frame_rate()->set_target(SENSOR_FRAME_RATE);
	            
	            this->scan_pattern.mutable_pulse()->set_frame_mode (blickfeld::protocol::config::ScanPattern::Pulse::FrameMode::ScanPattern_Pulse_FrameMode_COMBINE_UP_DOWN);
	            
	            #ifdef DISTORTION_CORRECTION
	            this->scan_pattern.mutable_pulse()->set_distortion_correction(true);
	            #endif
	            
	            #ifndef DISTORTION_CORRECTION
	            this->scan_pattern.mutable_pulse()->set_distortion_correction(false);	            
	            #endif
	            
	            #ifdef EQUI_ANGLE_MODE
	            this->scan_pattern.mutable_pulse()->set_type(blickfeld::protocol::config::ScanPattern::Pulse::Type::ScanPattern_Pulse_Type_EQUI_HORIZONTAL_ANGLE);
	            #endif
	            
	            #ifdef INTERLEAVE_MODE
	            this->scan_pattern.mutable_pulse()->set_type(blickfeld::protocol::config::ScanPattern::Pulse::Type::ScanPattern_Pulse_Type_INTERLEAVE);
	            #endif
            }
            
            blickfeld::protocol::config::ScanPattern get_pattern() {
                return this->scan_pattern;
            }
            
            void set_pattern(blickfeld::protocol::config::ScanPattern scan_pattern) {
                this->scan_pattern = scan_pattern;
            }
    };
    
    #ifdef USE_FILTERING
    class PointCloudFilter {
        private:
    	    blickfeld::protocol::config::ScanPattern::Filter filter;

    	public:
    	    PointCloudFilter() {
    	        std::cout << "Use filtering..." << std::endl;
        	    this->filter.mutable_range()->set_minimum(MIN_RANGE);
	            this->filter.mutable_range()->set_maximum(MAX_RANGE/0);
	            this->filter.set_max_number_of_returns_per_point(MAX_NUMBER_OF_RETURNS);
	            this->filter.set_delete_points_without_returns(DELETE_NO_RETURNS);
	        }
	        
	        blickfeld::protocol::config::ScanPattern::Filter get_filter() {
	            return this->filter;
	        }
    };
    #endif
    
    class BlickfeldLiDAR {
        private:
            std::shared_ptr<blickfeld::scanner> scanner;
            std::shared_ptr<blickfeld::point_cloud_stream<blickfeld::protocol::data::Frame>> stream_frame;
            const blickfeld::protocol::data::Frame reference_frame = blickfeld::scanner::REF_FRAME_XYZ_I_ID;
            
            // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ = new pcl::PointCloud<pcl::PointXYZI>;
            // sensor_msgs::msg::PointCloud2::SharedPtr message_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
            
            bool running = false;
            
            bool init_connection(std::string scanner_ip) {
	            try {
		            this->scanner = blickfeld::scanner::connect(scanner_ip);
		            std::cout << "Connected to LiDAR at address " << scanner_ip << std::endl;
		            return true;
	            } catch (const std::exception& e) {
	                std::cout << "Error occured while connecting to LiDAR: " << e.what() << std::endl;
		            return false;
	            }
            }
    
        public:
            BlickfeldLiDAR() {
                this->running = this->init_connection(SCANNER_IP);
            }
            
            #ifdef CUSTOM_SCAN_PATTERN
            bool complete_scan_pattern(ScanPattern* scan_pattern) {
                if (!this->running) return false;
                
                try {
		            scan_pattern->set_pattern(this->scanner->fill_scan_pattern(scan_pattern->get_pattern()));
		            std::cout << "Successfully completed scan pattern." << std::endl;
		            return true;
	            } catch (const blickfeld::protocol_exception<blickfeld::protocol::Error_NotInRange>& e) {
	                std::cout << "Error occured while completing scan pattern: " << e.what() << std::endl;
		            return false;
	            }
            }
            
            bool set_scan_pattern(ScanPattern* scan_pattern) {
                if (!this->running) return false;
                try {
		            this->scanner->set_scan_pattern(scan_pattern->get_pattern());
		            std::cout << "Successfully set scan pattern." << std::endl;
		            return true;
	            } catch (const blickfeld::protocol_exception<blickfeld::protocol::Error_InvalidRequest>& e) {
	                std::cout << "Error occured while setting scan pattern: " << e.what() << std::endl;
		            return false;
	            }
            }
            #endif
            
            #ifdef NAMED_SCAN_PATTERN
            bool list_named_scan_patterns() {
                if (!this->running) return false;
                try {
                    auto named_scan_patterns = this->scanner->get_named_scan_patterns();

	                // Print if the scan pattern is a default or custom named scan pattern
	                for (auto const& scan_pattern : named_scan_patterns) {
		                if (scan_pattern.read_only()) {
			                std::cout << "'" << scan_pattern.name() << "' is a default named scan pattern." << std::endl;
		                } else {
			                std::cout << "'" <<  scan_pattern.name() << "' is a custom named scan pattern."<< std::endl;
		                }
	                }
	        		return true;
	            } catch (const blickfeld::protocol_exception<blickfeld::protocol::Error_InvalidRequest>& e) {
	                std::cout << "Error occured while setting named scan pattern: " << e.what() << std::endl;
		            return false;
	            }
            }
            
            bool set_scan_pattern(std::string named_scan_pattern) {
                if (!this->running) return false;
                try {
		            this->scanner->set_scan_pattern(names_scan_pattern);
		            std::cout << "Successfully set scan pattern." << std::endl;
		            return true;
	            } catch (const blickfeld::protocol_exception<blickfeld::protocol::Error_InvalidRequest>& e) {
	                std::cout << "Error occured while setting named scan pattern: " << e.what() << std::endl;
		            return false;
	            }
            }
            #endif
            
            bool prepare_point_cloud_stream() {
                if (!this->running) return false;
                this->stream_frame = this->scanner->get_point_cloud_stream();
                return true;
            }
            
            #ifdef USE_FILTERING
            bool prepare_point_cloud_stream(PointCloudFilter* point_cloud_filter) {
                if (!this->running) return false;
                this->stream_frame = this->scanner->get_point_cloud_stream(point_cloud_filter->get_filter(), reference_frame);
                return true;
            } 
            #endif   
            
            std::shared_ptr<blickfeld::point_cloud_stream<blickfeld::protocol::data::Frame>> get_point_cloud_stream() {
                return this->stream_frame;
            }
            
            std::shared_ptr<blickfeld::scanner> get_scanner() {
                return this->scanner;
            }
            
            bool is_running() {
                return this->running;
            }
            
            void terminate_stream() {
                this->running = false;
                this->stream_frame = nullptr;
            }
    };
    
    #ifdef STREAM_IMU
    class BlickfeldIMU {
        private:
            std::shared_ptr<blickfeld::imu_stream> stream_imu;
            bool running = false;
    
        public:
            BlickfeldIMU(BlickfeldLiDAR * lidar) {
                this->running = lidar->is_running();
                if (this->running) {
                    this->stream_imu = lidar->get_scanner()->get_imu_stream();
                }
            }
            
            std::shared_ptr<blickfeld::imu_stream> get_imu_stream() {
                return this->stream_imu;
            }

            void terminate_stream() {
                this->running = false;
                this->stream_imu = nullptr;
            }
    };
    #endif
}
