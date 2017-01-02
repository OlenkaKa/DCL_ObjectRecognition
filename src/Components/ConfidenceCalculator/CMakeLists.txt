# Include the directory itself as a path to include directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
file(GLOB files *.cpp)

# Create an executable file from sources:
add_library(ConfidenceCalculator SHARED ${files})

# Link external libraries
target_link_libraries(ConfidenceCalculator ${DisCODe_LIBRARIES} ${Boost_LIBRARIES})

install_component(ConfidenceCalculator)
