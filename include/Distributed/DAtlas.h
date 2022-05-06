//
// Created by lz on 22-5-6.
//

#ifndef DISTRIBUTED_SLAM_DATLAS_H
#define DISTRIBUTED_SLAM_DATLAS_H

#include "Atlas/Map.h"
#include "Atlas/MapPoint.h"
#include "Atlas/KeyFrame.h"
#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"
#include "Visualise/Viewer.h"

#include <set>
#include <mutex>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>


using namespace ORB_SLAM3;
namespace D_SLAM {
//    class Viewer;
//
//    class Map;
//
//    class MapPoint;
//
//    class KeyFrame;
//
//    class KeyFrameDatabase;
//
//    class Frame;
//
//    class KannalaBrandt8;
//
//    class Pinhole;
//
////BOOST_CLASS_EXPORT_GUID(Pinhole, "Pinhole")
////BOOST_CLASS_EXPORT_GUID(KannalaBrandt8, "KannalaBrandt8")

    class D_Atlas {
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar.template register_type<Pinhole>();
            ar.template register_type<KannalaBrandt8>();

            // Save/load a set structure, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
            //ar & mspMaps;
            ar & mvpBackupMaps;
            ar & mvpCameras;
            // Need to save/load the static Id from Frame, KeyFrame, MapPoint and Map
            ar & Map::nNextId;
            ar & Frame::nNextId;
            ar & KeyFrame::nNextId;
            ar & MapPoint::nNextId;
            ar & GeometricCamera::nNextId;
            ar & mnLastInitKFidMap;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        D_Atlas();

        D_Atlas(int initKFid); // When its initialization the first map is created
        ~D_Atlas();

        void CreateNewMap();

        void ChangeMap(Map *pMap);

        unsigned long int GetLastInitKFid();

        void SetViewer(Viewer *pViewer);

        // Method for change components in the current map
        void AddKeyFrame(KeyFrame *pKF);

        void AddMapPoint(MapPoint *pMP);
        //void EraseMapPoint(MapPoint* pMP);
        //void EraseKeyFrame(KeyFrame* pKF);

        GeometricCamera *AddCamera(GeometricCamera *pCam);

        std::vector<GeometricCamera *> GetAllCameras();

        /* All methods without Map pointer work on current map */
        void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);

        void InformNewBigChange();

        int GetLastBigChangeIdx();

        long unsigned int MapPointsInMap();

        long unsigned KeyFramesInMap();

        // Method for get data in current map
        std::vector<KeyFrame *> GetAllKeyFrames();

        std::vector<MapPoint *> GetAllMapPoints();

        std::vector<MapPoint *> GetReferenceMapPoints();

        vector<Map *> GetAllMaps();

        int CountMaps();

        void clearMap();

        void clearAtlas();

        Map *GetCurrentMap();

        void SetMapBad(Map *pMap);

        void RemoveBadMaps();

        bool isInertial();

        void SetInertialSensor();

        void SetImuInitialized();

        bool isImuInitialized();

        // Function for garantee the correction of serialization of this object
        void PreSave();

        void PostLoad();

        map<long unsigned int, KeyFrame *> GetAtlasKeyframes();

        void SetKeyFrameDababase(KeyFrameDatabase *pKFDB);

        KeyFrameDatabase *GetKeyFrameDatabase();

        void SetORBVocabulary(ORBVocabulary *pORBVoc);

        ORBVocabulary *GetORBVocabulary();

        long unsigned int GetNumLivedKF();

        long unsigned int GetNumLivedMP();

    protected:

        std::set<Map *> mspMaps;
        std::set<Map *> mspBadMaps;
        // Its necessary change the container from set to vector because libboost 1.58 and Ubuntu 16.04 have an error with this cointainer
        std::vector<Map *> mvpBackupMaps;

        std::vector<Atlas> mvpSubAtlases;

        Map *mpCurrentMap;

        std::vector<GeometricCamera *> mvpCameras;

        unsigned long int mnLastInitKFidMap;

        Viewer *mpViewer;
        bool mHasViewer;

        // Class references for the map reconstruction from the save file
        KeyFrameDatabase *mpKeyFrameDB;
        ORBVocabulary *mpORBVocabulary;

        // Mutex
        std::mutex mMutexAtlas;


    }; // class Atlas

} // namespace ORB_SLAM3


#endif //DISTRIBUTED_SLAM_DATLAS_H
