cmake_minimum_required(VERSION 3.2)

# Add depthai-core dependency
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/depthai-core EXCLUDE_FROM_ALL)

set(TARGET_NAME test-app)
project(${TARGET_NAME})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

find_path(LIBUSB_INCLUDE_DIR NAMES libusb.h PATH_SUFFIXES "include" "libusb" "libusb-1.0")
find_library(LIBUSB_LIBRARY NAMES usb-1.0 PATH_SUFFIXES "lib")


# Add source files
add_executable("${TARGET_NAME}"
        src/rgb_video.cpp)

# Link with libraries
target_link_libraries(${TARGET_NAME}
        PRIVATE
        depthai::opencv
        ${CMAKE_THREAD_LIBS_INIT}
        PUBLIC ${LIBUSB_LIBRARY}
        PUBLIC ${OpenCV_LIBS}
        )

target_include_directories(${TARGET_NAME}
        PUBLIC "${LIBUSB_INCLUDE_DIR}"
        )

target_compile_options(${TARGET_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Werror=return-type>)
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 14)