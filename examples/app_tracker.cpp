/**
* This file is part of LDVO.
*
* Copyright 2019 Robert Maier, Technical University of Munich.
* For more information see <https://github.com/robmaier/ldvo>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* LDVO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LDVO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LDVO. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <CLI/CLI.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ldvo/dataset_tum.hpp>
#include <ldvo/timer.hpp>
#include <ldvo/timed_pose.hpp>
#include <ldvo/tracker.hpp>
#include <ldvo/version.hpp>


/**
 * @brief   LDVO tracker example application.
 *          Estimates the camera trajectory for a sequence from
 *          the TUM RGB-D benchmark through frame-to-frame tracking.
 * @author  Robert Maier
 */
int main(int argc, char *argv[])
{
    // parse command line arguments using CLI11
    CLI::App app("LDVO - Lightweight Dense Visual Odometry");

    // dataset parameters
    ldvo::DatasetTUM::Config data_cfg;
    app.add_option("-i,--input", data_cfg.dataset_folder, "Input dataset folder (TUM RGB-D benchmark)")->check(CLI::ExistingDirectory);
    app.add_option("-c,--cam_params", data_cfg.intrinsics_file, "Camera intrinsics parameters");
    app.add_option("--max_frames", data_cfg.max_num_frames, "Maximum number of frames to load");
    app.add_option("--max_depth", data_cfg.max_depth, "Maximum depth (e.g. 2.0)");
    // output parameters
    std::string poses_file = "";
    app.add_option("-p,--poses", poses_file, "Output camera poses (TUM RGB-D benchmark format)");
    // tracker parameters
    ldvo::Tracker::Config tracker_cfg;
    app.add_option("-l,--levels", tracker_cfg.num_levels, "Number of image pyramid levels");
    app.add_option("--min_level", tracker_cfg.min_level, "Minimum pyramid level");
    app.add_option("--max_level", tracker_cfg.max_level, "Maximum pyramid level");
    app.add_option("--iterations", tracker_cfg.num_iterations, "Maximum number of iterations per level");
    app.add_option("--update_thres", tracker_cfg.update_thres, "Update threshold for optimizer break");

    // parse command line arguments
    CLI11_PARSE(app, argc, argv);
    // validate tracker config to make sure it is valid
    tracker_cfg.validate();

    // print LDVO version
    std::cout << "LDVO version: " << ldvo::version() << std::endl;
    // print parameters
    data_cfg.print();
    std::cout << "output poses: " << poses_file << std::endl;
    tracker_cfg.print();

    // load TUM RGB-D benchmark dataset
    ldvo::DatasetTUM data(data_cfg);
    if (!data.init())
    {
        std::cerr << "Could not initialize dataset!" << std::endl;
        return 1;
    }
    data.camera().print();

    // container for storing pose graph (poses from current frame to world)
    std::vector<ldvo::TimedPose> poses;

    // create image pyramids
    const int w = data.camera().width();
    const int h = data.camera().height();
    const int num_levels = tracker_cfg.num_levels;
    std::shared_ptr<ldvo::FramePyramid> prev_pyramid =
            std::make_shared<ldvo::FramePyramid>(w, h, num_levels);
    std::shared_ptr<ldvo::FramePyramid> cur_pyramid =
            std::make_shared<ldvo::FramePyramid>(w, h, num_levels);

    // create tracker
    ldvo::Tracker tracker(tracker_cfg, data.camera());

    // load and downsample first frame
    std::shared_ptr<ldvo::Frame> prev_frame = data.loadFrame(0);
    prev_pyramid->fill(*prev_frame);

    // store initial camera pose for first frame
    Eigen::Matrix4f pose_cur_to_world = Eigen::Matrix4f::Identity();
    poses.push_back(ldvo::TimedPose(pose_cur_to_world, prev_frame->timeDepth()));

    // process frames
    std::cout << "processing " << data.numFrames() << " frames ..." << std::endl;
    double runtime_process = 0.0;
    double runtime_avg = 0.0;
    int num_frames_processed = 0;
    for (size_t i = 1; i < data.numFrames(); ++i)
    {
        std::cout << "   aligning frames " << (i-1) << " and " << i  << std::endl;

        // load frame
        std::shared_ptr<ldvo::Frame> cur_frame = data.loadFrame(i);
        // measure timing for processing new frame
        ldvo::Timer tmr_process;
        tmr_process.start();
        // downsample frame
        cur_pyramid->fill(*cur_frame);

        // measure timing for alignment
        ldvo::Timer tmr_align;
        tmr_align.start();

        // align previous frame to current
        Eigen::Matrix4f pose_prev_to_cur = Eigen::Matrix4f::Identity();
        bool ok = tracker.align(*prev_pyramid, *cur_pyramid, pose_prev_to_cur);

        // accumulate runtimes
        tmr_align.stop();
        runtime_avg += tmr_align.elapsed();
        tmr_process.stop();
        runtime_process += tmr_process.elapsed();
        ++num_frames_processed;

        if (ok)
        {
            // concatenate poses and store pose
            pose_cur_to_world = pose_cur_to_world * pose_prev_to_cur.inverse();
            poses.push_back(ldvo::TimedPose(pose_cur_to_world, cur_frame->timeDepth()));

            // switch previous and current frame
            std::swap(prev_pyramid, cur_pyramid);
        }
        else
        {
            std::cerr << "could not align frames " << (i-1) << " and " << i << "!" << std::endl;
        }
    }
    std::cout << "runtime alignment: " << (runtime_avg / num_frames_processed) * 1000.0 << " ms" << std::endl;
    std::cout << "runtime process + alignment: " << (runtime_process / num_frames_processed) * 1000.0 << " ms" << std::endl;
    std::cout << "frames aligned: " << poses.size() << std::endl;

    // save poses
    if (!poses_file.empty())
    {
        std::cout << "saving camera poses to " << poses_file << " ..." << std::endl;
        if (!ldvo::TimedPose::save(poses_file, poses))
            std::cerr << "Could not save camera poses!" << std::endl;
    }

    // clean up
    cv::destroyAllWindows();
    std::cout << "Direct Image Alignment finished." << std::endl;

    return 0;
}
