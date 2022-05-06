//
// Created by lz on 22-5-6.
//

#include "Distributed/DAtlas.h"


#include "Atlas/Atlas.h"
#include "Visualise/Viewer.h"

#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

namespace D_SLAM {

    D_Atlas::D_Atlas() {
        mpCurrentMap = static_cast<Map *>(NULL);
    }

    D_Atlas::D_Atlas(int initKFid) : mnLastInitKFidMap(initKFid), mHasViewer(false) {
        mpCurrentMap = static_cast<Map *>(NULL);
        CreateNewMap();
    }

    D_Atlas::~D_Atlas() {
        for (std::set<Map *>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;) {
            Map *pMi = *it;

            if (pMi) {
                delete pMi;
                pMi = static_cast<Map *>(NULL);

                it = mspMaps.erase(it);
            } else
                ++it;

        }
    }

    void D_Atlas::CreateNewMap() {
        unique_lock<mutex> lock(mMutexAtlas);
        cout << "Creation of new map with id: " << Map::nNextId << endl;
        if (mpCurrentMap) {
            if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
                mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() + 1; //The init KF is the next of current maximum

            mpCurrentMap->SetStoredMap();
            cout << "Stored map with ID: " << mpCurrentMap->GetId() << endl;

            //if(mHasViewer)
            //    mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
        }
        cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;

        mpCurrentMap = new Map(mnLastInitKFidMap);
        mpCurrentMap->SetCurrentMap();
        mspMaps.insert(mpCurrentMap);
    }

    void D_Atlas::ChangeMap(Map *pMap) {
        unique_lock<mutex> lock(mMutexAtlas);
        cout << "Change to map with id: " << pMap->GetId() << endl;
        if (mpCurrentMap) {
            mpCurrentMap->SetStoredMap();
        }

        mpCurrentMap = pMap;
        mpCurrentMap->SetCurrentMap();
    }

    unsigned long int D_Atlas::GetLastInitKFid() {
        unique_lock<mutex> lock(mMutexAtlas);
        return mnLastInitKFidMap;
    }

    void D_Atlas::SetViewer(Viewer *pViewer) {
        mpViewer = pViewer;
        mHasViewer = true;
    }

    void D_Atlas::AddKeyFrame(KeyFrame *pKF) {
        Map *pMapKF = pKF->GetMap();
        pMapKF->AddKeyFrame(pKF);
    }

    void D_Atlas::AddMapPoint(MapPoint *pMP) {
        Map *pMapMP = pMP->GetMap();
        pMapMP->AddMapPoint(pMP);
    }

    GeometricCamera *D_Atlas::AddCamera(GeometricCamera *pCam) {
        //Check if the camera already exists
        bool bAlreadyInMap = false;
        int index_cam = -1;
        for (size_t i = 0; i < mvpCameras.size(); ++i) {
            GeometricCamera *pCam_i = mvpCameras[i];
            if (!pCam) std::cout << "Not pCam" << std::endl;
            if (!pCam_i) std::cout << "Not pCam_i" << std::endl;
            if (pCam->GetType() != pCam_i->GetType())
                continue;

            if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
                if (((Pinhole *) pCam_i)->IsEqual(pCam)) {
                    bAlreadyInMap = true;
                    index_cam = i;
                }
            } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
                if (((KannalaBrandt8 *) pCam_i)->IsEqual(pCam)) {
                    bAlreadyInMap = true;
                    index_cam = i;
                }
            }
        }

        if (bAlreadyInMap) {
            return mvpCameras[index_cam];
        } else {
            mvpCameras.push_back(pCam);
            return pCam;
        }
    }

    std::vector<GeometricCamera *> D_Atlas::GetAllCameras() {
        return mvpCameras;
    }

    void D_Atlas::SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs) {
        unique_lock<mutex> lock(mMutexAtlas);
        mpCurrentMap->SetReferenceMapPoints(vpMPs);
    }

    void D_Atlas::InformNewBigChange() {
        unique_lock<mutex> lock(mMutexAtlas);
        mpCurrentMap->InformNewBigChange();
    }

    int D_Atlas::GetLastBigChangeIdx() {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetLastBigChangeIdx();
    }

    long unsigned int D_Atlas::MapPointsInMap() {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->MapPointsInMap();
    }

    long unsigned D_Atlas::KeyFramesInMap() {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->KeyFramesInMap();
    }

    std::vector<KeyFrame *> D_Atlas::GetAllKeyFrames() {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetAllKeyFrames();
    }

    std::vector<MapPoint *> D_Atlas::GetAllMapPoints() {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetAllMapPoints();
    }

    std::vector<MapPoint *> D_Atlas::GetReferenceMapPoints() {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->GetReferenceMapPoints();
    }

    vector<Map *> D_Atlas::GetAllMaps() {
        unique_lock<mutex> lock(mMutexAtlas);
        struct compFunctor {
            inline bool operator()(Map *elem1, Map *elem2) {
                return elem1->GetId() < elem2->GetId();
            }
        };
        vector<Map *> vMaps(mspMaps.begin(), mspMaps.end());
        sort(vMaps.begin(), vMaps.end(), compFunctor());
        return vMaps;
    }

    int D_Atlas::CountMaps() {
        unique_lock<mutex> lock(mMutexAtlas);
        return mspMaps.size();
    }

    void D_Atlas::clearMap() {
        unique_lock<mutex> lock(mMutexAtlas);
        mpCurrentMap->clear();
    }

    void D_Atlas::clearAtlas() {
        unique_lock<mutex> lock(mMutexAtlas);
        /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
        {
            (*it)->clear();
            delete *it;
        }*/
        mspMaps.clear();
        mpCurrentMap = static_cast<Map *>(NULL);
        mnLastInitKFidMap = 0;
    }

    Map *D_Atlas::GetCurrentMap() {
        unique_lock<mutex> lock(mMutexAtlas);
        if (!mpCurrentMap)
            CreateNewMap();
        while (mpCurrentMap->IsBad())
            usleep(3000);

        return mpCurrentMap;
    }

    void D_Atlas::SetMapBad(Map *pMap) {
        mspMaps.erase(pMap);
        pMap->SetBad();

        mspBadMaps.insert(pMap);
    }

    void D_Atlas::RemoveBadMaps() {
        /*for(Map* pMap : mspBadMaps)
        {
            delete pMap;
            pMap = static_cast<Map*>(NULL);
        }*/
        mspBadMaps.clear();
    }

    bool D_Atlas::isInertial() {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->IsInertial();
    }

    void D_Atlas::SetInertialSensor() {
        unique_lock<mutex> lock(mMutexAtlas);
        mpCurrentMap->SetInertialSensor();
    }

    void D_Atlas::SetImuInitialized() {
        unique_lock<mutex> lock(mMutexAtlas);
        mpCurrentMap->SetImuInitialized();
    }

    bool D_Atlas::isImuInitialized() {
        unique_lock<mutex> lock(mMutexAtlas);
        return mpCurrentMap->isImuInitialized();
    }

    void D_Atlas::PreSave() {
        if (mpCurrentMap) {
            if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
                mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() + 1; //The init KF is the next of current maximum
        }

        struct compFunctor {
            inline bool operator()(Map *elem1, Map *elem2) {
                return elem1->GetId() < elem2->GetId();
            }
        };
        std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
        sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

        std::set<GeometricCamera *> spCams(mvpCameras.begin(), mvpCameras.end());
        for (Map *pMi: mvpBackupMaps) {
            if (!pMi || pMi->IsBad())
                continue;

            if (pMi->GetAllKeyFrames().size() == 0) {
                // Empty map, erase before of save it.
                SetMapBad(pMi);
                continue;
            }
            pMi->PreSave(spCams);
        }
        RemoveBadMaps();
    }

    void D_Atlas::PostLoad() {
        map<unsigned int, GeometricCamera *> mpCams;
        for (GeometricCamera *pCam: mvpCameras) {
            mpCams[pCam->GetId()] = pCam;
        }

        mspMaps.clear();
        unsigned long int numKF = 0, numMP = 0;
        for (Map *pMi: mvpBackupMaps) {
            mspMaps.insert(pMi);
            pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams);
            numKF += pMi->GetAllKeyFrames().size();
            numMP += pMi->GetAllMapPoints().size();
        }
        mvpBackupMaps.clear();
    }

    void D_Atlas::SetKeyFrameDababase(KeyFrameDatabase *pKFDB) {
        mpKeyFrameDB = pKFDB;
    }

    KeyFrameDatabase *D_Atlas::GetKeyFrameDatabase() {
        return mpKeyFrameDB;
    }

    void D_Atlas::SetORBVocabulary(ORBVocabulary *pORBVoc) {
        mpORBVocabulary = pORBVoc;
    }

    ORBVocabulary *D_Atlas::GetORBVocabulary() {
        return mpORBVocabulary;
    }

    long unsigned int D_Atlas::GetNumLivedKF() {
        unique_lock<mutex> lock(mMutexAtlas);
        long unsigned int num = 0;
        for (Map *pMap_i: mspMaps) {
            num += pMap_i->GetAllKeyFrames().size();
        }

        return num;
    }

    long unsigned int D_Atlas::GetNumLivedMP() {
        unique_lock<mutex> lock(mMutexAtlas);
        long unsigned int num = 0;
        for (Map *pMap_i: mspMaps) {
            num += pMap_i->GetAllMapPoints().size();
        }

        return num;
    }

    map<long unsigned int, KeyFrame *> D_Atlas::GetAtlasKeyframes() {
        map<long unsigned int, KeyFrame *> mpIdKFs;
        for (Map *pMap_i: mvpBackupMaps) {
            vector<KeyFrame *> vpKFs_Mi = pMap_i->GetAllKeyFrames();

            for (KeyFrame *pKF_j_Mi: vpKFs_Mi) {
                mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
            }
        }

        return mpIdKFs;
    }

} //namespace ORB_SLAM3

