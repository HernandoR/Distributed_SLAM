
#include "Visualise/Viewer.h"
#include <pangolin/pangolin.h>
#include "Settings.h"

#include <mutex>


namespace ORB_SLAM3 {

//    Viewer::Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking,
//                   const string &strSettingPath, Settings *settings) :
//            both(false), mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
//            mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false) {
//        if (settings) {
//            newParameterLoader(settings);
//        } else {
//
//            cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
//
//            bool is_correct = ParseViewerParamFile(fSettings);
//
//            if (!is_correct) {
//                std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
//                try {
//                    throw -1;
//                }
//                catch (std::exception &e) {
//
//                }
//            }
//        }
//
//        mbStopTrack = false;
//    }
    atomic<int> Viewer::smpWindow_ID = 0;
//    atomic<bool> Viewer::isCreating = false;
    std::mutex Viewer::mMutexCreateWin = std::mutex();


    Viewer::Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking,
                   const string &strSettingPath, Settings *settings) :
            both(false), mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
            mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false) {
        if (settings) {
            newParameterLoader(settings);
        } else {

            cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

            bool is_correct = ParseViewerParamFile(fSettings);

            if (!is_correct) {
                std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
                try {
                    throw -1;
                }
                catch (std::exception &e) {
                }
            }
        }

        mbStopTrack = false;
        mpWindow_ID = smpWindow_ID;
        smpWindow_ID++;
    }

    Viewer::Viewer(const Viewer &) {
        smpWindow_ID++;
    }

    void Viewer::newParameterLoader(Settings *settings) {
        mImageViewerScale = 1.f;

        float fps = settings->fps();
        if (fps < 1)
            fps = 30;
        mT = 1e3 / fps;

        cv::Size imSize = settings->newImSize();
        mImageHeight = imSize.height;
        mImageWidth = imSize.width;

        mImageViewerScale = settings->imageViewerScale();
        mViewpointX = settings->viewPointX();
        mViewpointY = settings->viewPointY();
        mViewpointZ = settings->viewPointZ();
        mViewpointF = settings->viewPointF();
    }

    bool Viewer::ParseViewerParamFile(cv::FileStorage &fSettings) {
        bool b_miss_params = false;
        mImageViewerScale = 1.f;

        float fps = fSettings["Camera.fps"];
        if (fps < 1)
            fps = 30;
        mT = 1e3 / fps;

        cv::FileNode node = fSettings["Camera.width"];
        if (!node.empty()) {
            mImageWidth = node.real();
        } else {
            std::cerr << "*Camera.width parameter doesn't exist or is not a real number* .." << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.height"];
        if (!node.empty()) {
            mImageHeight = node.real();
        } else {
            std::cerr << "*Camera.height parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.imageViewScale"];
        if (!node.empty()) {
            mImageViewerScale = node.real();
        }

        node = fSettings["Viewer.ViewpointX"];
        if (!node.empty()) {
            mViewpointX = node.real();
        } else {
            std::cerr << "*Viewer.ViewpointX parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.ViewpointY"];
        if (!node.empty()) {
            mViewpointY = node.real();
        } else {
            std::cerr << "*Viewer.ViewpointY parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.ViewpointZ"];
        if (!node.empty()) {
            mViewpointZ = node.real();
        } else {
            std::cerr << "*Viewer.ViewpointZ parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Viewer.ViewpointF"];
        if (!node.empty()) {
            mViewpointF = node.real();
        } else {
            std::cerr << "*Viewer.ViewpointF parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        return !b_miss_params;
    }

    void Viewer::Run() {
        mbFinished = false;
        mbStopped = false;
        cout << "starting pangolin viewer on thread " + to_string(mpWindow_ID) << endl;
        mMutexCreateWin.lock();

        pangolin::CreateWindowAndBind("ORB-SLAM3: Map Viewer " + to_string(mpWindow_ID), 1024, 768);
        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", false, true);
        pangolin::Var<bool> menuCamView("menu.Camera View", false, false);
        pangolin::Var<bool> menuTopView("menu.Top View", false, false);
        // pangolin::Var<bool> menuSideView("menu.Side View",false,false);
        pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph", false, true);
        pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph", true, true);
        pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
        pangolin::Var<bool> menuReset("menu.Reset", false, false);
        pangolin::Var<bool> menuStop("menu.Stop", false, false);
        pangolin::Var<bool> menuStepByStep("menu.Step By Step", false, true);  // false, true
        pangolin::Var<bool> menuStep("menu.Step", false, false);

        pangolin::Var<bool> menuShowOptLba("menu.Show LBA opt", false, true);
        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
                pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
        );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::OpenGlMatrix Twc, Twr;
        Twc.SetIdentity();
        pangolin::OpenGlMatrix Ow; // Oriented with g in the z axis
        Ow.SetIdentity();
        cv::namedWindow("ORB-SLAM3: Current Frame " + to_string(mpWindow_ID));

        bool bFollow = true;
        bool bLocalizationMode = false;
        bool bStepByStep = false;
        bool bCameraView = true;

        if (mpTracker->mSensor == mpSystem->MONOCULAR || mpTracker->mSensor == mpSystem->STEREO ||
            mpTracker->mSensor == mpSystem->RGBD) {
            menuShowGraph = true;
        }

        float trackedImageScale = mpTracker->GetImageScale();

        mMutexCreateWin.unlock();
        cout << "Starting the Viewer" << endl;
        while (1) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc, Ow);

            if (mbStopTrack) {
                menuStepByStep = true;
                mbStopTrack = false;
            }

            if (menuFollowCamera && bFollow) {
                if (bCameraView)
                    s_cam.Follow(Twc);
                else
                    s_cam.Follow(Ow);
            } else if (menuFollowCamera && !bFollow) {
                if (bCameraView) {
                    s_cam.SetProjectionMatrix(
                            pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1,
                                                       1000));
                    s_cam.SetModelViewMatrix(
                            pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0,
                                                      0.0));
                    s_cam.Follow(Twc);
                } else {
                    s_cam.SetProjectionMatrix(
                            pangolin::ProjectionMatrix(1024, 768, 3000, 3000, 512, 389, 0.1, 1000));
                    s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0.01, 10, 0, 0, 0, 0.0, 0.0, 1.0));
                    s_cam.Follow(Ow);
                }
                bFollow = true;
            } else if (!menuFollowCamera && bFollow) {
                bFollow = false;
            }

            if (menuCamView) {
                menuCamView = false;
                bCameraView = true;
                s_cam.SetProjectionMatrix(
                        pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 10000));
                s_cam.SetModelViewMatrix(
                        pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0,
                                                  0.0));
                s_cam.Follow(Twc);
            }

            if (menuTopView && mpMapDrawer->mpAtlas->isImuInitialized()) {
                menuTopView = false;
                bCameraView = false;
                s_cam.SetProjectionMatrix(
                        pangolin::ProjectionMatrix(1024, 768, 3000, 3000, 512, 389, 0.1, 10000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0.01, 50, 0, 0, 0, 0.0, 0.0, 1.0));
                s_cam.Follow(Ow);
            }

            if (menuLocalizationMode && !bLocalizationMode) {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            } else if (!menuLocalizationMode && bLocalizationMode) {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }

            if (menuStepByStep && !bStepByStep) {
                //cout << "Viewer: step by step" << endl;
                mpTracker->SetStepByStep(true);
                bStepByStep = true;
            } else if (!menuStepByStep && bStepByStep) {
                mpTracker->SetStepByStep(false);
                bStepByStep = false;
            }

            if (menuStep) {
                mpTracker->mbStep = true;
                menuStep = false;
            }


            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            mpMapDrawer->DrawCurrentCamera(Twc);
            if (menuShowKeyFrames || menuShowGraph || menuShowInertialGraph || menuShowOptLba)
                mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph, menuShowInertialGraph,
                                           menuShowOptLba);
            if (menuShowPoints)
                mpMapDrawer->DrawMapPoints();

            pangolin::FinishFrame();

            cv::Mat toShow;
            cv::Mat im = mpFrameDrawer->DrawFrame(trackedImageScale);

            if (both) {
                cv::Mat imRight = mpFrameDrawer->DrawRightFrame(trackedImageScale);
                cv::hconcat(im, imRight, toShow);
            } else {
                toShow = im;
            }

            if (mImageViewerScale != 1.f) {
                int width = toShow.cols * mImageViewerScale;
                int height = toShow.rows * mImageViewerScale;
                cv::resize(toShow, toShow, cv::Size(width, height));
            }

            cv::imshow("ORB-SLAM3: Current Frame " + to_string(mpWindow_ID), toShow);
            cv::waitKey(mT);

            if (menuReset) {
                menuShowGraph = true;
                menuShowInertialGraph = true;
                menuShowKeyFrames = true;
                menuShowPoints = true;
                menuLocalizationMode = false;
                if (bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
                bFollow = true;
                menuFollowCamera = true;
                mpSystem->ResetActiveMap();
                menuReset = false;
            }

            if (menuStop) {
                if (bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();

                // Stop all threads
                mpSystem->Shutdown();

                // Save camera trajectory
                mpSystem->SaveTrajectoryEuRoC("CameraTrajectory.txt");
                mpSystem->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
                menuStop = false;
            }

            if (Stop()) {
                while (isStopped()) {
                    usleep(3000);
                }
            }

            if (CheckFinish())
                break;
        }

        SetFinish();
    }

    void Viewer::RequestFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop() {
        unique_lock<mutex> lock(mMutexStop);
        if (!mbStopped)
            mbStopRequested = true;
    }

    bool Viewer::isStopped() {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::Stop() {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);

        if (mbFinishRequested)
            return false;
        else if (mbStopRequested) {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;

    }

    void Viewer::Release() {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
    }

    Viewer::~Viewer() {
        smpWindow_ID--;
    }
/*void Viewer::SetTrackingPause()
{
    mbStopTrack = true;
}*/

}
