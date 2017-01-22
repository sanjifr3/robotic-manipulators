cmake_minimum_required(VERSION 2.8.3)
project(video_stream_opencv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
)


find_package(OpenCV)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(video_stream src/video_stream.cpp)
target_link_libraries(video_stream ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(invertColor src/invertColor.cpp)
target_link_libraries(invertColor ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(simpleCanny src/simpleCanny.cpp)
target_link_libraries(simpleCanny ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(video_capture src/video_capture.cpp)
#target_link_libraries(video_capture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(my_subscriber src/my_subscriber.cpp)
#target_link_libraries(my_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(imagePublisher src/imagePublisher.cpp)
target_link_libraries(imagePublisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(hough src/hough.cpp)
target_link_libraries(hough ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


#add_executable(BlobDetection src/BlobDetection.cpp)
#target_link_libraries(BlobDetection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(my_publisher_simple src/my_publisher_simple.cpp)
# target_link_libraries(my_publisher_simple ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS video_stream
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
  scripts/test_video_resource.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
