/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

void StartSLAMSystem(const string vocFile, const string settingsFile, unsigned long nImages,
                     vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv) {
    if (argc != 4) {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence " << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;

    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);
    // path_to_sequence should have at least 3 sub intents : times.txt img0 img1
    // see LoadImages in this file for detail

    auto SPLIT_NUM = 2;
    const auto nImages = vstrImageLeft.size();
    const int subNum = ceil(nImages / SPLIT_NUM);

    vector<vector<string>> subLeftImg(SPLIT_NUM), subRightImg(SPLIT_NUM);
    vector<vector<double>> subvTimestamps(SPLIT_NUM);
    vector<std::thread> thread_pool(SPLIT_NUM);


//    StartSLAMSystem(argv[1], argv[2], nImages, vstrImageLeft, vstrImageRight, vTimestamps);

    for (int i = 0; i < SPLIT_NUM; i++) {

        if (i < (SPLIT_NUM - 1)) {
            subLeftImg[i].assign(vstrImageLeft.begin() + i * subNum, vstrImageLeft.begin() + (i + 1) * subNum);
            subRightImg[i].assign(vstrImageRight.begin() + i * subNum, vstrImageRight.begin() + (i + 1) * subNum);
            subvTimestamps[i].assign(vTimestamps.begin() + i * subNum, vTimestamps.begin() + (i + 1) * subNum);

        } else {
            subLeftImg[i].assign(vstrImageLeft.begin() + i * subNum, vstrImageLeft.end());
            subRightImg[i].assign(vstrImageRight.begin() + i * subNum, vstrImageRight.end());
            subvTimestamps[i].assign(vTimestamps.begin() + i * subNum, vTimestamps.end());
        }


    }
//    thread_pool[1] = std::thread(StartSLAMSystem, argv[1], argv[2], nImages,
//                                 std::ref(vstrImageLeft), std::ref(vstrImageRight), std::ref(vTimestamps));

    int i = 0;

    for (auto &lthr: thread_pool) {
        unsigned long nums = subLeftImg[i].size();
//        if (i == 1) {
        thread_pool[i] = std::thread(StartSLAMSystem, argv[1], argv[2], nums,
                                     std::ref(subLeftImg[i]), std::ref(subRightImg[i]),
                                     std::ref(subvTimestamps[i]));
//        }
        i++;
    }

    cout << "started " << SPLIT_NUM << " thread" << endl <<
         endl;

//
    for (
        auto &lthr
            : thread_pool) {
        if (lthr.

                joinable()

                )
            lthr.

                    join();

    }
    return 0;

};

void StartSLAMSystem(const string vocFile, const string settingsFile, unsigned long nImages,
                     vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimestamps) {



    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocFile, settingsFile, ORB_SLAM3::System::STEREO, true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    double t_track = 0.f;
    double t_resize = 0.f;

    // Main loop
    cv::Mat imLeft, imRight;
//    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    auto start_time_point = std::chrono::steady_clock::now();
    for (int ni = 0; ni < nImages; ni++) {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];    // time of which img were taken

        if (imLeft.empty()) {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
//            return 1;
        }

        if (imageScale != 1.f)
            // TODO: 1.f ?
        {
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC17
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
            // time stamp
#else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
#endif
#endif
            // resize
            // todo: seems have to specific round method
            // or maybe not that important
            int width = imLeft.cols * imageScale;
            int height = imLeft.rows * imageScale;
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC17
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
#endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            // ? time used to resize pic, by milisec
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef COMPILEDWITHC17
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft, imRight, tframe);

#ifdef COMPILEDWITHC17
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame

        // what is it waiting for?
        // it seems that it calculates the time it has used to tack, compare with which has been available,
        // sleep the overlap. so that it simulates
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1];

        if (ttrack < T)
            usleep((T - ttrack) * 1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++) {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;
    cout << "total processing Time: " << totaltime << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

//    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps) {
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for (int i = 0; i < nTimes; i++) {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
