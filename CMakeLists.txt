cmake_minimum_required(VERSION 3.10)

project(kinefix VERSION 1.4.0 LANGUAGES C)

option(KINEFIX_BUILD_TESTS "Build kinefix tests" OFF)
option(KINEFIX_PORTABLE_WIDE "Force portable two-limb wide arithmetic" OFF)
option(KINEFIX_SANITIZERS "Enable address and undefined behavior sanitizers" OFF)
option(KINEFIX_BUILD_BENCHMARKS "Build deterministic load scenario" OFF)
option(KINEFIX_INSTALL "Install kinefix" OFF)

add_library(kinefix STATIC
    src/kinefix.c
    src/kinefix_geometry.c
    src/kinefix_world.c
    src/kinefix_queries.c
    src/kinefix_character.c
    src/kinefix_dynamics.c
    src/kinefix_rotation.c)
add_library(kinefix::kinefix ALIAS kinefix)
target_include_directories(kinefix PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
set_target_properties(kinefix PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
set(kinefix_targets kinefix)
if(KINEFIX_SANITIZERS)
    if(MSVC)
        message(FATAL_ERROR "Combined ASan/UBSan requires Clang or GCC")
    endif()
    target_compile_options(kinefix PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_libraries(kinefix PUBLIC -fsanitize=address,undefined)
endif()

if(KINEFIX_BUILD_TESTS)
    enable_testing()
    foreach(suite tests geometry_tests world_tests character_tests dynamics_tests query_tests bounds_tests ccd_tests rotation_tests)
        add_executable(kinefix_${suite} tests/kinefix_${suite}.c)
        target_link_libraries(kinefix_${suite} PRIVATE kinefix)
        set_target_properties(kinefix_${suite} PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
        add_test(NAME kinefix_${suite} COMMAND kinefix_${suite})
        list(APPEND kinefix_targets kinefix_${suite})
    endforeach()
endif()

if(KINEFIX_BUILD_BENCHMARKS)
    foreach(benchmark kinefix_benchmark kinefix_level_benchmark)
        add_executable(${benchmark} benchmarks/${benchmark}.c)
        target_link_libraries(${benchmark} PRIVATE kinefix)
        set_target_properties(${benchmark} PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
        list(APPEND kinefix_targets ${benchmark})
    endforeach()
    if(KINEFIX_BUILD_TESTS)
        add_test(NAME kinefix_load_determinism
            COMMAND ${CMAKE_COMMAND} -DEXECUTABLE=$<TARGET_FILE:kinefix_benchmark>
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/load_determinism.cmake)
    endif()
endif()

foreach(target IN LISTS kinefix_targets)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    if(KINEFIX_PORTABLE_WIDE)
        target_compile_definitions(${target} PRIVATE KF_PORTABLE_WIDE=1)
    endif()
endforeach()

if(KINEFIX_INSTALL)
    install(TARGETS kinefix ARCHIVE DESTINATION lib)
    install(DIRECTORY include/kinefix DESTINATION include)
endif()
