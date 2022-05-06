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


#ifndef DVIEWER_H
#define DVIEWER_H

#include "Visualise/FrameDrawer.h"
#include "Visualise/MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "Settings.h"

#include <mutex>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace std;

namespace ORB_SLAM3 {

    class Tracking;

    class FrameDrawer;

    class MapDrawer;

    class System;

    class Settings;

    class Viewer;

    class DViewer : public Viewer {
    public:
        DViewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking,
                const string &strSettingPath, Settings *settings);

//        DViewer(const Viewer & )
        ~DViewer();

        void Run() override;


    private:
        vector<Viewer> mvpViewer;


    };

}


#endif // DVIEWER_H


