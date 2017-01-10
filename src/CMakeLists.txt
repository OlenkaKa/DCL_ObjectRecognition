set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(.)

# ##############################################################################
# Find required libraries
# ##############################################################################

# Find Boost, at least ver. 1.41
find_package(Boost 1.41.0 REQUIRED COMPONENTS system thread filesystem date_time)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

# Set variable with list of all libraries common for this DCL
set(DCL_LIBRARIES "${DisCODe_LIBRARIES} ${Boost_LIBRARIES}")

# Find another necessary libraries
find_package(PCL 1.7.2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(DCL_LIBRARIES ${DCL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# Find another DCLs our one depends on
discode_find_dcl(CvBasic)
discode_find_dcl(CvCoreTypes)
discode_find_dcl(PCLCoreTypes)
discode_find_dcl(SIFTObjectModel)

# Check, whether all necessary libraries are linked
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,defs")

# Add C++11 support
if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()
# ##############################################################################
# Add source directories
# ##############################################################################

# CvBlobs components
add_subdirectory(Components)

# CvBlobs types
add_subdirectory(Types)

# Prepare config file to use from another DCLs
configure_file(ObjectRecognitionConfig.cmake.in ${CMAKE_INSTALL_PREFIX}/ObjectRecognitionConfig.cmake @ONLY)

