//
// Created by lz on 22-5-6.
//

#ifndef DISTRIBUTED_SLAM_VIEWER_H
#define DISTRIBUTED_SLAM_VIEWER_H

#include "Visualise/FrameDrawer.h"
#include "Visualise/MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "Settings.h"
#include "Visualise/Viewer.h"

#include <mutex>
#include <Eigen/Core>
#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace std;

namespace ORB_SLAM3 {

    class Tracking;

    class FrameDrawer;

    class MapDrawer;

    class System;

    class Settings;

    class Distributed_Viewer {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Distributed_Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
                           Tracking *pTracking, const string &strSettingPath, Settings *settings);

        void newParameterLoader(Settings *settings);

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
        // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
        void Run();

        void RequestFinish();

        void RequestStop();

        bool isFinished();

        bool isStopped();

        bool isStepByStep();

        void Release();

        //void SetTrackingPause();

        bool both;
    private:

        bool ParseViewerParamFile(cv::FileStorage &fSettings);

        bool Stop();

        System *mpSystem;
        FrameDrawer *mpFrameDrawer;
        MapDrawer *mpMapDrawer;
        Tracking *mpTracker;

        // 1/fps in ms
        double mT;
        float mImageWidth, mImageHeight;
        float mImageViewerScale;

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

        bool CheckFinish();

        void SetFinish();

        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        std::mutex mMutexStop;

        bool mbStopTrack;

    };
}


#endif //DISTRIBUTED_SLAM_VIEWER_H
