#include "blickfeld_features.hpp"
#include "blickfeld_functions.hpp"

#include <nora_framework/node.hpp>
#include <nora_framework/metadata/metadata_manager.hpp>
#include <nora_framework/metadata/capability.hpp>

#include <rclcpp/rclcpp.hpp>
#include <iostream>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <blickfeld/scanner.h>
#include <blickfeld/utils.h>
#include <blickfeld/discover.h>
#include <blickfeld/protocol_exception.h>

#include <signal.h>

bool running = true;

void my_sigint_handler(int sig) {
    running = false;
}

std::shared_ptr<LifecycleStateManager> lifecycle_manager_;

void OnNodeExecuted(NORA::Framework::Node* node) {
    lifecycle_manager_ = LifecycleStateManager::GetStateManager();

    // Capability wird erstellt. Diese Methode fügt die Capability direkt dem MetadataManager hinzu.
    // Der Typ der Capability (also der Typ des zugehörigen Topics) ist std_msgs::msg::String, kann
    // aber jeder Typ sein, der auch als Typ von Publishern / Subscribern benutzt werden kann.
    NORA::Framework::Capability<sensor_msgs::msg::PointCloud2>* DemoCapability =
        NORA::Framework::Capability<sensor_msgs::msg::PointCloud2>::FromConfig("lidar_pointcloud");

    // Die Node wird nach dem Erstellen aller Capabilities und Dependencies als
    // konfiguriert markiert. Daraufhin wird diese "inactive" geschaltet und
    // erfüllt dann wenn möglich selbstständig alle (harten) Abhängigkeiten
    // und geht dann "active".
    auto mdm = node->GetMetadataManager();
    mdm->SetConfigured(true);

    std::thread business_logic([&DemoCapability]() {
        // Erstelle eine neue Nachricht, damit sie nachher versendet werden kann.    
        // Dies ist eine Funktion aus ROS2, nicht aus dem Framework        

        blickfeld_functions::BlickfeldLiDAR * lidar = new blickfeld_functions::BlickfeldLiDAR();
	    blickfeld_functions::ScanPattern * scan_pattern = new blickfeld_functions::ScanPattern();
	    
	    #ifdef CUSTOM_SCAN_PATTERN
	    lidar->set_scan_pattern(scan_pattern);
	    lidar->complete_scan_pattern(scan_pattern);
	    #endif
	    
	    #ifdef NAMED_SCAN_PATTERN
	    lidar->list_named_scan_pattern();
	    lidar->set_scan_pattern(NAMED_SCAN_PATTERN_ID);
	    #endif
	    
	    #ifdef USE_FILTERING
	    blickfeld_functions::PointCloudFilter * filter = new blickfeld_functions::PointCloudFilter();
	    lidar->prepare_point_cloud_stream(filter);
        #endif
        
	    #ifndef USE_FILTERING
	    lidar->repare_point_cloud_stream();
	    #endif
	    
	    #ifdef STREAM_IMU
	    blickfeld_functions::BlickfeldIMU * imu = new blickfeld_functions::BlickfeldIMU(lidar);
	    #endif
	    
	    // Synchronous request for scan_pattern
	    printf("Scan pattern: %s", lidar->get_scanner()->get_scan_pattern().DebugString().c_str());

	    // Synchronous request for status
	    printf("Sensor status: %s", lidar->get_scanner()->get_status().DebugString().c_str());
	    
	    float x_avg, y_avg, z_avg, intensity_avg;
	    int frame_counter = 0;
	    
        //pcl::PointCloud<pcl::PointXYZI> cloud_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
      	sensor_msgs::msg::PointCloud2::SharedPtr message_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
      	
        while(running) {
		    // Request frame. It has a reduced data size, as not all fields are requested. This also speeds up the Protobuf encoding & decoding.
		    // Format of frame is described in protocol/blickfeld/data/frame.proto or doc/protocol.md
		    // Protobuf API is described in https://developers.google.com/protocol-buffers/docs/cpptutorial
		    const blickfeld::protocol::data::Frame frame = lidar->get_point_cloud_stream()->recv_frame();

		    // Print information about this frame
		    std::cout << "Frame information: " << frame << std::endl;

		    cloud_->points.clear();

		    // Example for scanline and point iteration
		    // Iterate through all the scanlines in a frame
		    for (int s_ind = 0; s_ind < frame.scanlines_size(); s_ind++) {
			    // Iterate through all the points in a scanline
			    // std::cout << frame.scanlines_size() << " - " << frame.scanlines(s_ind).points_size() << std::endl;
			    for (int p_ind = 0; p_ind < frame.scanlines(s_ind).points_size(); p_ind++) {
				    auto& point = frame.scanlines(s_ind).points(p_ind);

				    // Iterate through all the returns for each points
				    x_avg = 0.0f; y_avg = 0.0f; z_avg = 0.0f; intensity_avg = 0.0f;
				    for (int r_ind = 0; r_ind < point.returns_size(); r_ind++) {
					    auto& ret = point.returns(r_ind);
						    
					    x_avg += ret.cartesian(0);
					    y_avg += ret.cartesian(1);
					    z_avg += ret.cartesian(2);
					    intensity_avg += ret.intensity();
					    
					    
					    if (p_ind == 10 && s_ind == 10)
					    {
						    printf("Point %u -ret %u [x: %4.2f, y: %4.2f, z: %4.2f, range: %4.2f] - intensity: %u\n", point.id(), ret.id(), ret.cartesian(0), ret.cartesian(1), ret.cartesian(2), ret.range(), ret.intensity());
						    printf("Ambient: %u  Offset: %lu\n",  point.ambient_light_level(), point.start_offset_ns());

					    }
				     }
				     
				     pcl::PointXYZI pt = pcl::PointXYZI(intensity_avg);
				     pt.x = x_avg; pt.y = y_avg; pt.z = z_avg;
				     cloud_->points.push_back(pt);
			    }
		    }

		    pcl::toROSMsg(*cloud_, *message_);
		    message_->header.frame_id = "lidar_pointcloud";
		    // const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
		    // auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
		    // auto now_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
		    // message_->header.stamp = rclcpp::Time(now_sec, now_nanosec);
		    message_->header.stamp = lifecycle_manager_->GetNode()->now();
		    
            // Sendet die eben erstellte Nachricht mit den Framework-eigenen Methoden.
            DemoCapability->SendMessage(*message_);
                	
            RCLCPP_INFO(rclcpp::get_logger(__func__), "SendMessage");

		    frame_counter++;
		    if (frame_counter == 7)
		    {		    
			    // Synchronous request for status
			    printf("Sensor status: %s", lidar->get_scanner()->get_status().DebugString().c_str());
		    }
		    
		    if (frame_counter == 7)
		    {
		        frame_counter = 0;
			    #ifdef STREAM_IMU
			    const blickfeld::protocol::data::IMU data = imu->get_imu_stream()->recv_burst();
			    // Extract samples and print
			    int print_n_samples = 1;
			    for (auto sample : data.samples()) {
				    std::cout << "- acceleration: ["
					      << std::setprecision(3) << std::fixed
					      << sample.acceleration(0) << ", "
					      << sample.acceleration(1) << ", "
					      << sample.acceleration(2) << "], angular velocity: ["
					      << sample.angular_velocity(0) << ", "
					      << sample.angular_velocity(1) << ", "
					      << sample.angular_velocity(2) << "]>"
					      << std::endl;
				    if (!--print_n_samples)
					    break;
			    }
			    #endif
		    }
        }
        std::cout << "Exiting..." << std::endl;
        
        lidar->terminate_stream();
        #ifdef STREAM_IMU
        imu->terminate_stream();
        delete imu;
        #endif
        
        delete lidar;
        delete scan_pattern;
        #ifdef USE_FILTERING
        delete filter;
        #endif
    });

    node->SpinForever();
}

int main(int argc, const char* const* argv, char** envp) {
    NORA::Framework::Node provider_node(DEFAULT_NODE_NAME); // Erstelle eine neue Node mit dem Namen in DEFAULT_NODE_NAME
    signal(SIGINT, my_sigint_handler);
    provider_node.Execute(argc, argv, envp, OnNodeExecuted); // Führe die Node aus und übergebe Kommandozeilenargumente
    return 0;
}
