/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

void LoadGroundTruth(vector<cv::Mat> &groundTruthVec);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Set up the ICP algorithm...
    cout << endl << "Loading config file (config.yaml)..." << endl;
    LidarMono::Config caselitz_settings("config.yaml");

    LidarMono::LidarMap lidar_map(caselitz_settings);
    cout << "Loading LiDAR point cloud (map.pcd)..." << endl;
    lidar_map.loadPointCloud("map.pcd");

    cout << "Loading voxel map (map.bin)..." << endl << endl;
    lidar_map.loadMap("map.bin");
#if 1
    LidarMono::ICP icp(&lidar_map, caselitz_settings);
#else
    LidarMono::g2oICP icp(&lidar_map, caselitz_settings);
#endif

    // Loading ground truth data for initialization...
    vector<cv::Mat> groundTruth;
    LoadGroundTruth(groundTruth);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true, &icp, &lidar_map);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe,&groundTruth[ni]);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}

void LoadGroundTruth(vector<cv::Mat> &groundTruthVec)
{
    ifstream groundTruthFile;
    groundTruthFile.open("ground_truth.txt");
    while (!groundTruthFile.eof())
    {
        string line;
        getline(groundTruthFile, line);

        if (!line.empty()) {
            stringstream in(line);
            float nums[12];
            in  >> nums[0] >> nums[1] >> nums[2] >> nums[3]
                >> nums[4] >> nums[5] >> nums[6] >> nums[7]
                >> nums[8] >> nums[9] >> nums[10] >> nums[11];

            cv::Mat pose(4, 4, CV_32F);
            pose.at<float>(0, 0) = nums[0];
            pose.at<float>(0, 1) = nums[1];
            pose.at<float>(0, 2) = nums[2];
            pose.at<float>(0, 3) = nums[3];
            pose.at<float>(1, 0) = nums[4];
            pose.at<float>(1, 1) = nums[5];
            pose.at<float>(1, 2) = nums[6];
            pose.at<float>(1, 3) = nums[7];
            pose.at<float>(2, 0) = nums[8];
            pose.at<float>(2, 1) = nums[9];
            pose.at<float>(2, 2) = nums[10];
            pose.at<float>(2, 3) = nums[11];
            pose.at<float>(3, 0) = 0;
            pose.at<float>(3, 1) = 0;
            pose.at<float>(3, 2) = 0;
            pose.at<float>(3, 3) = 1;
            groundTruthVec.push_back(pose);
        }
    }
}
