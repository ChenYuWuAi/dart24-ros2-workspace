# add_compile_options(-O4)
# find opencv
find_package(OpenCV CONFIG REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# link lib 
link_directories(thirdparty/cameraDriver/DHCamera_arm64/lib)
include_directories(
    thirdparty/cameraDriver/DHCamera_arm64    ##here
)

target_link_libraries(
    ${OpenCV_LIBS}
    MVSDK
)

