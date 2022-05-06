##my applications
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Applications)

#add_executable(stereo_inertial_euroc
#        Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
#target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})
#
#add_executable(stereo_inertial_tum_vi
#        Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc)
#target_link_libraries(stereo_inertial_tum_vi ${PROJECT_NAME})
#
#if(realsense2_FOUND)
#    add_executable(stereo_inertial_realsense_t265
#            Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
#    target_link_libraries(stereo_inertial_realsense_t265 ${PROJECT_NAME})
#
#    add_executable(stereo_inertial_realsense_D435i
#            Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
#    target_link_libraries(stereo_inertial_realsense_D435i ${PROJECT_NAME})
#endif()
#
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Calibration)
#if(realsense2_FOUND)
#    add_executable(recorder_realsense_D435i
#            Examples/Calibration/recorder_realsense_D435i.cc)
#    target_link_libraries(recorder_realsense_D435i ${PROJECT_NAME})
#
#    add_executable(recorder_realsense_T265
#            Examples/Calibration/recorder_realsense_T265.cc)
#    target_link_libraries(recorder_realsense_T265 ${PROJECT_NAME})
#endif()