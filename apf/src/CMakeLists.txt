set(BUILD_SHARED_LIBS ON)

set(SRC_FILES
        geometry/Capsule.cpp
        geometry/Line.cpp
        transform/transform.cpp
        controller/Controller.cpp
        transform/jacobian.cpp
        apf/apf.cpp
        gateway.h
)

# Define the library target
add_library(apf_hgg_core SHARED ${SRC_FILES})

# Include directories for the library
target_include_directories(apf_hgg_core PUBLIC
        ${CMAKE_SOURCE_DIR}/lib
        ${CMAKE_SOURCE_DIR}/src
)

# Set C++ standard requirement
target_compile_features(apf_hgg_core PUBLIC cxx_std_23)