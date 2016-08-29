# Include the directory itself as a path to include directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
file(GLOB files *.cpp)

# Find required packages
#find_package(OpenCV REQUIRED)
#find_package(REQUIRED)

# Create an executable file from sources:
add_library(MatchCorrespondences SHARED ${files})

# Link external libraries
#target_link_libraries(MatchCorrespondences ${DisCODe_LIBRARIES})
target_link_libraries(MatchCorrespondences ${DisCODe_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

install_component(MatchCorrespondences)
