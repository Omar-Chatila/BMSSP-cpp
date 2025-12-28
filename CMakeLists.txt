cmake_minimum_required(VERSION 3.18)
project(algo_seminar LANGUAGES CXX)

# ======================
# Language standard
# ======================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ======================
# Header-only library
# ======================
add_library(algo_seminar INTERFACE)

target_include_directories(algo_seminar
        INTERFACE
        ${PROJECT_SOURCE_DIR}/src
)

# Optional: show headers in IDE
target_sources(algo_seminar INTERFACE
        src/FibHeap.h
        src/FibHeap.tpp
        src/Graph.cpp
        src/Dijkstra.cpp
)

# ======================
# OpenMP (optional)
# ======================
find_package(OpenMP)
if (OpenMP_CXX_FOUND)
    target_link_libraries(algo_seminar
            INTERFACE
            OpenMP::OpenMP_CXX
    )
endif()

# ======================
# Executable
# ======================
add_executable(algo_seminar_main main.cpp)
target_link_libraries(algo_seminar_main PRIVATE algo_seminar)

# ======================
# Doxygen (optional)
# ======================
find_package(Doxygen OPTIONAL_COMPONENTS dot)

if (DOXYGEN_FOUND)
    set(DOXYGEN_IN  ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    add_custom_target(doc
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating Doxygen documentation"
            VERBATIM
    )
endif()
