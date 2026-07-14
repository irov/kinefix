cmake_minimum_required(VERSION 3.10)

project(kinefix VERSION 1.0.0 LANGUAGES C)

option(KINEFIX_BUILD_TESTS "Build kinefix tests" OFF)
option(KINEFIX_INSTALL "Install kinefix" OFF)

add_library(kinefix STATIC src/kinefix.c)
add_library(kinefix::kinefix ALIAS kinefix)
target_include_directories(kinefix PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
set_target_properties(kinefix PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

if(MSVC)
    target_compile_options(kinefix PRIVATE /W4 /WX)
else()
    target_compile_options(kinefix PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

if(KINEFIX_BUILD_TESTS)
    enable_testing()
    add_executable(kinefix_tests tests/kinefix_tests.c)
    target_link_libraries(kinefix_tests PRIVATE kinefix)
    set_target_properties(kinefix_tests PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
    add_test(NAME kinefix_tests COMMAND kinefix_tests)
endif()

if(KINEFIX_INSTALL)
    install(TARGETS kinefix ARCHIVE DESTINATION lib)
    install(DIRECTORY include/kinefix DESTINATION include)
endif()
