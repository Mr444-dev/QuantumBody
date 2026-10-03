cmake_minimum_required(VERSION 3.21)
project(QuantumBody VERSION 0.1.0 LANGUAGES CXX)

option(QUANTUMBODY_BUILD_TESTS "Build the small deterministic verification suite" ON)
option(QUANTUMBODY_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

add_library(quantumbody
    src/world.cpp
    src/scenes.cpp
)
add_library(QuantumBody::quantumbody ALIAS quantumbody)
target_include_directories(quantumbody PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(quantumbody PUBLIC cxx_std_20)
if(MSVC)
    target_compile_options(quantumbody PRIVATE /W4 /permissive-)
else()
    target_compile_options(quantumbody PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

if(QUANTUMBODY_ENABLE_SANITIZERS AND NOT MSVC)
    target_compile_options(quantumbody PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(quantumbody PUBLIC -fsanitize=address,undefined)
endif()

add_executable(quantumbody_demo src/main.cpp)
target_link_libraries(quantumbody_demo PRIVATE QuantumBody::quantumbody)
set_target_properties(quantumbody_demo PROPERTIES OUTPUT_NAME quantumbody)

if(QUANTUMBODY_BUILD_TESTS)
    enable_testing()
    add_executable(quantumbody_tests tests/physics_tests.cpp)
    target_link_libraries(quantumbody_tests PRIVATE QuantumBody::quantumbody)
    add_test(NAME quantumbody.physics COMMAND quantumbody_tests)
endif()
