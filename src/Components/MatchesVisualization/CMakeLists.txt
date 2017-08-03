# Include the directory itself as a path to include directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_compile_options(-std=c++11)

# Create a variable containing all .cpp files:
file(GLOB files *.cpp)

# Find required packages
#find_package(OpenCV REQUIRED)

# Create an executable file from sources:
add_library(MatchesVisualization SHARED ${files})

# Link external libraries
target_link_libraries(MatchesVisualization ${DisCODe_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

install_component(MatchesVisualization)
