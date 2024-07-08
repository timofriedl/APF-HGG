include_directories(${CMAKE_SOURCE_DIR}/lib)
include_directories(${CMAKE_SOURCE_DIR}/src)

add_executable(UnitTests
        LineTest.cpp
        JacobianTest.cpp
        TransformTest.cpp
        CapsuleTest.cpp
)

target_link_libraries(UnitTests
        apf_hgg_core
        gtest
        gtest_main
)

set_target_properties(UnitTests PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

include(GoogleTest)
gtest_discover_tests(UnitTests)