if(WITH_OPENCV)
  set(OpenCV_DIR /home/jyi/Devel/OpenCV/2412/share/OpenCV)
  find_package(OpenCV REQUIRED)
  if(OpenCV_FOUND)
    set(OPENCV_LIBRARIES "${OpenCV_LIBS}")
    set(OpenCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS}
                            /home/jyi/Devel/OpenCV/2412/include)
    set(OPENCV_INCLUDE_DIRS "${OpenCV_INCLUDE_DIRS}")
    set(HAVE_OPENCV TRUE)
  endif()
endif()

