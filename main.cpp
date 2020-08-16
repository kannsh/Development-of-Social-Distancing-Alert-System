// Copyright (C) 2018 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#include "core.hpp"
#include "utils.hpp"
#include "tracker.hpp"
#include "descriptor.hpp"
#include "distance.hpp"
#include "detector.hpp"
#include "image_reader.hpp"
#include "pedestrian_tracker_demo.hpp"

#include <opencv2/core.hpp>
#include "homography.h"

#include <iostream>
#include <utility>
#include <vector>
#include <map>
#include <memory>
#include <string>
#include <gflags/gflags.h>


using namespace InferenceEngine;
using ImageWithFrameIndex = std::pair<cv::Mat, int>;

std::unique_ptr<PedestrianTracker>
CreatePedestrianTracker(const std::string& reid_model,
                        const std::string& reid_weights,
                        const InferenceEngine::InferencePlugin& reid_plugin,
                        bool should_keep_tracking_info) {
    TrackerParams params;

    if (should_keep_tracking_info) {
        params.drop_forgotten_tracks = false;
        params.max_num_objects_in_track = -1;
    }

    std::unique_ptr<PedestrianTracker> tracker(new PedestrianTracker(params));

    // Load reid-model.
    std::shared_ptr<IImageDescriptor> descriptor_fast =
        std::make_shared<ResizedImageDescriptor>(
            cv::Size(16, 32), cv::InterpolationFlags::INTER_LINEAR);
    std::shared_ptr<IDescriptorDistance> distance_fast =
        std::make_shared<MatchTemplateDistance>();

    tracker->set_descriptor_fast(descriptor_fast);
    tracker->set_distance_fast(distance_fast);

    if (!reid_model.empty() && !reid_weights.empty()) {
        CnnConfig reid_config(reid_model, reid_weights);
        reid_config.max_batch_size = 16;

        std::shared_ptr<IImageDescriptor> descriptor_strong =
            std::make_shared<DescriptorIE>(reid_config, reid_plugin);

        if (descriptor_strong == nullptr) {
            THROW_IE_EXCEPTION << "[SAMPLES] internal error - invalid descriptor";
        }
        std::shared_ptr<IDescriptorDistance> distance_strong =
            std::make_shared<CosDistance>(descriptor_strong->size());

        tracker->set_descriptor_strong(descriptor_strong);
        tracker->set_distance_strong(distance_strong);
    } else {
        std::cout << "WARNING: Either reid model or reid weights "
            << "were not specified. "
            << "Only fast reidentification approach will be used." << std::endl;
    }

    return tracker;
}

bool ParseAndCheckCommandLine(int argc, char *argv[]) {
    // ---------------------------Parsing and validation of input args--------------------------------------

    gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
    if (FLAGS_h) {
        showUsage();
        return false;
    }

    std::cout << "[ INFO ] Parsing input parameters" << std::endl;

    if (FLAGS_i.empty()) {
        throw std::logic_error("Parameter -i is not set");
    }

    if (FLAGS_m_det.empty()) {
        throw std::logic_error("Parameter -m_det is not set");
    }

    if (FLAGS_m_reid.empty()) {
        throw std::logic_error("Parameter -m_reid is not set");
    }

    return true;
}



int main_work(int argc, char **argv) {
    std::cout << "InferenceEngine: " << GetInferenceEngineVersion() << std::endl;
    
    if (!ParseAndCheckCommandLine(argc, argv)) {
        return 0;
    }


    // Reading command line parameters.
    auto video_path = FLAGS_i;

    auto det_model = FLAGS_m_det;
    auto det_weights = fileNameNoExt(FLAGS_m_det) + ".bin";

    auto reid_model = FLAGS_m_reid;
    auto reid_weights = fileNameNoExt(FLAGS_m_reid) + ".bin";

    auto detlog_out = FLAGS_out;

    auto detector_mode = FLAGS_d_det;
    auto reid_mode = FLAGS_d_reid;

    auto custom_cpu_library = FLAGS_l;
    auto path_to_custom_layers = FLAGS_c;
    bool should_use_perf_counter = FLAGS_pc;

    bool should_print_out = FLAGS_r;

    bool should_show = !FLAGS_no_show;
    int delay = FLAGS_delay;
    if (!should_show)
        delay = -1;
    should_show = (delay >= 0);

    int first_frame = FLAGS_first;
    int last_frame = FLAGS_last;

    bool should_save_det_log = !detlog_out.empty();

    if (first_frame >= 0)
        std::cout << "first_frame = " << first_frame << std::endl;
    if (last_frame >= 0)
        std::cout << "last_frame = " << last_frame << std::endl;

    std::vector<std::string> devices{detector_mode, reid_mode};
    std::map<std::string, InferencePlugin> plugins_for_devices =
        LoadPluginForDevices(
            devices, custom_cpu_library, path_to_custom_layers,
            should_use_perf_counter);

    DetectorConfig detector_confid(det_model, det_weights);
    auto detector_plugin = plugins_for_devices[detector_mode];

    ObjectDetector pedestrian_detector(detector_confid, detector_plugin);

    auto reid_plugin = plugins_for_devices[reid_mode];

    bool should_keep_tracking_info = should_save_det_log || should_print_out;
    std::unique_ptr<PedestrianTracker> tracker =
        CreatePedestrianTracker(reid_model, reid_weights, reid_plugin,
                                should_keep_tracking_info);



    // Opening video.
    std::unique_ptr<ImageReader> video =
        ImageReader::CreateImageReaderForPath(video_path);

    PT_CHECK(video->IsOpened()) << "Failed to open video: " << video_path;
    double video_fps = video->GetFrameRate();


    // View FPS Value
    std::cout << "\n" << "FPS:" << video_fps << "\n";

    // Initialize Homography Transformation of selected area 
    setup_transformation_matrix();
    
    // imageLoc variables to draw area used in video 
    std::vector<cv::Point2f> imageLocs;
    imageLocs.push_back(cv::Point2f(591, 276)); 
    imageLocs.push_back(cv::Point2f(1897, 494));
    imageLocs.push_back(cv::Point2f(1672, 924)); 
    imageLocs.push_back(cv::Point2f(110, 480));  

    if (first_frame > 0)
        video->SetFrameIndex(first_frame);


    for (;;) {
        auto pair = video->Read();
        cv::Mat frame = pair.first;
        int frame_idx = pair.second; 
        

        if (frame.empty()) break;

        PT_CHECK(frame_idx >= first_frame);

        if ( (last_frame >= 0) && (frame_idx > last_frame) ) {
            std::cout << "Frame " << frame_idx << " is greater than last_frame = "
                << last_frame << " -- break";
            break;
        }

        pedestrian_detector.submitFrame(frame, frame_idx);
        pedestrian_detector.waitAndFetchResults();

        TrackedObjects detections = pedestrian_detector.getResults();

        // timestamp in milliseconds
        uint64_t cur_timestamp = 1000.0 / video_fps * frame_idx;
        tracker->Process(frame, detections, cur_timestamp);

        if (should_show) {
            // Drawing colored "worms" (tracks).
            frame = tracker->DrawActiveTracks(frame);
            
            // Do not draw all detected objects (We only want tracked detected objects)
            // Drawing all detected objects on a frame in GREEN 
            /*for (const auto &detection : detections) {
                cv::rectangle(frame, detection.rect, cv::Scalar(0, 255, 0), 3);
            }*/
            
            // Read worldmap
            cv::Mat worldmap = cv::imread("//home//student//Desktop//videomap.bmp"); 
            
            // Declare worldLocs (corresponding to imageLocs) 
            cv::Point2f idealPoint1 = cv::Point2f(0,0);      
            cv::Point2f idealPoint2 = cv::Point2f(1420,0);   
            cv::Point2f idealPoint3 = cv::Point2f(1380,800);
            cv::Point2f idealPoint4 = cv::Point2f(0,800);  

            // Drawing tracked detections only by RED color and print ID and detection
            // confidence level.
            for (const auto &detection : tracker->TrackedDetections()) {
                cv::rectangle(frame, detection.rect, cv::Scalar(0, 0, 255), 3);
                std::string text = std::to_string(detection.object_id) +
                    " conf: " + std::to_string(detection.confidence);
                cv::putText(frame, text, detection.rect.tl(), cv::FONT_HERSHEY_COMPLEX,      
                            1.0, cv::Scalar(0, 0, 255), 3); 

             
                // Draw imagepoints in video (Red box of Human Coordinates)
                cv::Point2f imgpoint = cv::Point2f(((detection.rect.br().x + detection.rect.tl().x) * 0.5), 
                                             detection.rect.br().y);  
                cv::circle(frame, imgpoint, 10, cv::Scalar(255,0,255), -1);

                
                // Draw imagepoints based on class Obj_p
                struct Obj_pixels Obj_p;
                Obj_p.iSource_id=2;
                Obj_p.iImage_x[0]=((detection.rect.br().x + detection.rect.tl().x) * 0.5);   
                Obj_p.iImage_y[0]=detection.rect.br().y;                                      
                
                // Transform imagepoints and get real-time world map coordinates of imagepoints
                Obj_p=TransformImage2Map(Obj_p);
             
                // Draw the transformed imagepoints(mappoint) on map  
                cv::Point2f mappoint = cv::Point2f(Obj_p.oMap_x[0], Obj_p.oMap_y[0]);
                cv::circle(worldmap, mappoint, 10, cv::Scalar(255,0,255), -1);
                
                // Draw the input area used in video on worldmap
                cv::circle(worldmap, idealPoint1, 10, cv::Scalar(212,175,55), -20);
                cv::circle(worldmap, idealPoint2, 10, cv::Scalar(212,175,55), -10);
                cv::circle(worldmap, idealPoint3, 10, cv::Scalar(212,175,55), -10);
                cv::circle(worldmap, idealPoint4, 10, cv::Scalar(212,175,55), -10); 
   
                // Output real world coordinates of human coordinates
                std::cout << "\ncamera id: " << Obj_p.iSource_id  << std::endl;
                std::cout << "real world coordinates of human: " << std::endl;
                std::cout << "x: " << Obj_p.oMap_x[0]  << std::endl;
                std::cout << "y: " << Obj_p.oMap_y[0]  << std::endl;

                // Output image pixel coordinate of human coordinates
                std::cout << "x pixel: " << Obj_p.iImage_x[0] << std::endl;
                std::cout << "y pixel: " << Obj_p.iImage_y[0] << std::endl;
            }	    
 
                // Output real world location area value used on map
                std::cout << "\n" << "Real World location area drawn on map:" << std::endl;
                std::cout << idealPoint1 << std::endl; 
                std::cout << idealPoint2 << std::endl; 
                std::cout << idealPoint3 << std::endl;  
                std::cout << idealPoint4 << std::endl;  


            // Draw input area used in video
            for(unsigned int i=0; i<imageLocs.size(); ++i)
            {
             	cv::circle(frame, imageLocs[i], 13, cv::Scalar(0,255,0), -1); 

                cv::line(frame, imageLocs[0], imageLocs[1], cv::Scalar(0,255,0), 8);
                cv::line(frame, imageLocs[1], imageLocs[2], cv::Scalar(0,255,0), 8);
                cv::line(frame, imageLocs[2], imageLocs[3], cv::Scalar(0,255,0), 8);
                cv::line(frame, imageLocs[3], imageLocs[0], cv::Scalar(0,255,0), 8);
            }     

            // Resize video and map to fit screen
            cv::resize(frame, frame, cv::Size(), 0.5, 0.45);  
            cv::resize(worldmap, worldmap, cv::Size(), 0.7, 0.7);

            // Display video & map
            cv::imshow("dbg", frame);
            cv::imshow("worldmap with points: ", worldmap);             
           
            char k = cv::waitKey(delay);
            if (k == 27)
                break;   
        }


        if (should_save_det_log && (frame_idx % 100 == 0)) {
            DetectionLog log = tracker->GetDetectionLog(true);
            SaveDetectionLogToTrajFile(detlog_out, log);
        }
    }

    if (should_keep_tracking_info) {
        DetectionLog log = tracker->GetDetectionLog(true);

        if (should_save_det_log)
            SaveDetectionLogToTrajFile(detlog_out, log);
        if (should_print_out)
            PrintDetectionLog(log);
    }
    if (should_use_perf_counter) {
        pedestrian_detector.PrintPerformanceCounts();
        tracker->PrintReidPerformanceCounts();
    }
    return 0;
}

int main(int argc, char **argv) {

    try {
        main_work(argc, argv);
    }
    catch (const std::exception& error) {
        std::cerr << "[ ERROR ] " << error.what() << std::endl;
        return 1;
    }
    catch (...) {
        std::cerr << "[ ERROR ] Unknown/internal exception happened." << std::endl;
        return 1;
    }

    std::cout << "[ INFO ] Execution successful" << std::endl;

    return 0;
}
