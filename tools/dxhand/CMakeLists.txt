cmake_minimum_required(VERSION 3.16)
project(dxhand LANGUAGES CXX)
include(CTest)
set(NATIVE "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/tsensei_retarget/src/a32_driver/native_linux")
# The SDK's combined test option also requires the full Web bridge example.
# Select just the original offline transport and protocol tests for this service.
function(add_native_driver)
    set(BUILD_TESTING OFF)
    add_subdirectory("${NATIVE}" a32)
endfunction()
add_native_driver()
# The transport context is private to its translation unit. Compile that same
# source once, with the two service-only ABI entries alongside it.
set_property(TARGET a32_native PROPERTY SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/transport.cpp" "${NATIVE}/protocol.cpp")
target_include_directories(a32_native PRIVATE "${NATIVE}")

if(BUILD_TESTING)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME linux_offline_transport COMMAND "${Python3_EXECUTABLE}"
        "${NATIVE}/tests/test_offline.py" "$<TARGET_FILE:a32_native>")
    add_executable(a3_test_protocol "${NATIVE}/../../../vendor/a3-cpp-sdk/tests/test_protocol.cpp")
    target_link_libraries(a3_test_protocol PRIVATE a3_sdk)
    add_test(NAME a3_test_protocol COMMAND a3_test_protocol)
endif()
