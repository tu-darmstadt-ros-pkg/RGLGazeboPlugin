
########################################################################
# RobotecGPULidar
########################################################################

set(RGL_PROJECT_DIR "${CMAKE_CURRENT_BINARY_DIR}/RobotecGPULidar")
set(RGL_LIBRARY_DIR "${RGL_PROJECT_DIR}/lib")
set(RGL_SO_FILENAME "libRobotecGPULidar.so")
set(RGL_SO_PATH "${RGL_LIBRARY_DIR}/${RGL_SO_FILENAME}")
set(RGL_INCLUDE_DIR "${RGL_PROJECT_DIR}/include")
set(RGL_API_HEADERS_DIR "${RGL_INCLUDE_DIR}/rgl/api")

set(RGL_CUSTOM_LIBRARY_PATH "" CACHE FILEPATH
    "Path to the custom RobotecGPULidar library build. An alternative to release downloading.")
set(RGL_CUSTOM_API_HEADER_PATH "" CACHE PATH
    "Path to the include directory with API headers compatible with the custom library build
     provided with `RGL_CUSTOM_LIBRARY_FILEPATH`.
     In most cases it will be path to `include` directory of RobotecGPULidar project")

# The plugin requires the colored-point-cloud RGL API (rgl_texture_create_rgba8888,
# rgl_entity_set_color_texture), which the stock RobotecAI releases do not provide.
# By default RGL is therefore cloned and built from source (requires the CUDA toolkit;
# the OptiX headers are fetched automatically from NVIDIA's official header-only repo).
set(RGL_BUILD_FROM_SOURCE ON CACHE BOOL
    "Clone and build RobotecGPULidar from source instead of downloading a prebuilt release.")
set(RGL_GIT_REPOSITORY "https://github.com/tu-darmstadt-ros-pkg/RobotecGPULidar.git" CACHE STRING
    "Git repository to build RobotecGPULidar from.")
set(RGL_GIT_TAG "color-pointcloud" CACHE STRING
    "Git branch/tag/commit of RobotecGPULidar to build.")
set(RGL_OPTIX_GIT_REPOSITORY "https://github.com/NVIDIA/optix-dev.git" CACHE STRING
    "Git repository providing the OptiX SDK headers (header-only, no NVIDIA login required).")
set(RGL_OPTIX_GIT_TAG "v7.2.0" CACHE STRING
    "OptiX headers version to fetch.")

if((RGL_CUSTOM_LIBRARY_PATH AND NOT RGL_CUSTOM_API_HEADER_PATH) OR
   (NOT RGL_CUSTOM_LIBRARY_PATH AND RGL_CUSTOM_API_HEADER_PATH))
    message(FATAL_ERROR "It is required to set both `RGL_CUSTOM_LIBRARY_PATH` and `RGL_CUSTOM_API_HEADER_PATH`\
                         to finish custom RobotecGPULidar configuration")
endif()

# Runs a command at configure time and aborts with a readable message on failure.
function(rgl_run_step step_name)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "RobotecGPULidar source build: step '${step_name}' failed (exit code ${result}).")
    endif()
endfunction()

if(RGL_CUSTOM_LIBRARY_PATH AND RGL_CUSTOM_API_HEADER_PATH) # Copy custom RobotecGPULidar from the provided location
    file(MAKE_DIRECTORY ${RGL_LIBRARY_DIR})
    file(COPY ${RGL_CUSTOM_LIBRARY_PATH} DESTINATION ${RGL_LIBRARY_DIR})
    file(COPY ${RGL_CUSTOM_API_HEADER_PATH} DESTINATION ${RGL_PROJECT_DIR})
elseif(RGL_BUILD_FROM_SOURCE) # Clone and build RobotecGPULidar from source
    # Rebuild from scratch when the configured sources change, so an existing build
    # directory cannot silently keep linking an RGL built from different refs.
    set(RGL_SOURCES_STAMP "${RGL_GIT_REPOSITORY};${RGL_GIT_TAG};${RGL_OPTIX_GIT_REPOSITORY};${RGL_OPTIX_GIT_TAG}")
    set(RGL_STAMP_FILE "${CMAKE_CURRENT_BINARY_DIR}/rgl-sources.stamp")
    set(RGL_SRC_DIR "${CMAKE_CURRENT_BINARY_DIR}/RobotecGPULidar-src")
    set(OPTIX_DIR "${CMAKE_CURRENT_BINARY_DIR}/optix-dev")
    if(EXISTS ${RGL_STAMP_FILE})
        file(READ ${RGL_STAMP_FILE} RGL_PREVIOUS_STAMP)
        if(NOT RGL_PREVIOUS_STAMP STREQUAL RGL_SOURCES_STAMP)
            message(STATUS "RobotecGPULidar source configuration changed; rebuilding from scratch.")
            file(REMOVE_RECURSE ${RGL_SRC_DIR} ${OPTIX_DIR} ${RGL_PROJECT_DIR})
            file(REMOVE ${RGL_STAMP_FILE})
        endif()
    endif()

    if(NOT EXISTS ${RGL_SO_PATH})
        set(RGL_BUILD_DIR "${RGL_SRC_DIR}/build")

        # OptiX is header-only at build time (the implementation ships with the NVIDIA
        # driver), so cloning the official header repo replaces a manual SDK install.
        if(NOT EXISTS "${OPTIX_DIR}/include/optix.h")
            message(STATUS "Fetching OptiX headers ${RGL_OPTIX_GIT_TAG} from ${RGL_OPTIX_GIT_REPOSITORY}...")
            rgl_run_step("clone optix headers"
                git clone --depth 1 --branch ${RGL_OPTIX_GIT_TAG} ${RGL_OPTIX_GIT_REPOSITORY} ${OPTIX_DIR})
        endif()

        # Plain clone + checkout so RGL_GIT_TAG may be a branch, tag, or commit SHA.
        if(NOT EXISTS "${RGL_SRC_DIR}/CMakeLists.txt")
            message(STATUS "Cloning RobotecGPULidar (${RGL_GIT_TAG}) from ${RGL_GIT_REPOSITORY}...")
            rgl_run_step("clone RobotecGPULidar"
                git clone ${RGL_GIT_REPOSITORY} ${RGL_SRC_DIR})
            rgl_run_step("checkout RobotecGPULidar ref"
                git -C ${RGL_SRC_DIR} checkout ${RGL_GIT_TAG})
        endif()

        # Clones RGL's third-party dependencies (spdlog, yaml-cpp, ...); idempotent.
        message(STATUS "Installing RobotecGPULidar dependencies...")
        rgl_run_step("install RGL dependencies"
            ${CMAKE_COMMAND} -E chdir ${RGL_SRC_DIR} python3 install_deps.py)

        # The CUDA tools (nvcc, bin2c) are often not on PATH with plain cuda-toolkit
        # installations; prepend the standard location so CMake's CUDA language
        # detection and RGL's find_program(bin2c) succeed.
        set(RGL_BUILD_ENV "OptiX_INSTALL_DIR=${OPTIX_DIR}")
        if(EXISTS "/usr/local/cuda/bin/nvcc")
            list(APPEND RGL_BUILD_ENV "PATH=/usr/local/cuda/bin:$ENV{PATH}")
        endif()

        message(STATUS "Building RobotecGPULidar from source (this can take a few minutes)...")
        rgl_run_step("configure RobotecGPULidar"
            ${CMAKE_COMMAND} -E env ${RGL_BUILD_ENV}
            ${CMAKE_COMMAND} -S ${RGL_SRC_DIR} -B ${RGL_BUILD_DIR}
                -DCMAKE_BUILD_TYPE=Release
                -DRGL_BUILD_TESTS=OFF
                -DRGL_BUILD_TOOLS=OFF)
        include(ProcessorCount)
        ProcessorCount(RGL_NPROC)
        if(RGL_NPROC EQUAL 0)
            set(RGL_NPROC 4)
        endif()
        rgl_run_step("build RobotecGPULidar"
            ${CMAKE_COMMAND} --build ${RGL_BUILD_DIR} --target RobotecGPULidar --parallel ${RGL_NPROC})

        file(MAKE_DIRECTORY ${RGL_LIBRARY_DIR})
        file(COPY "${RGL_BUILD_DIR}/lib/${RGL_SO_FILENAME}" DESTINATION ${RGL_LIBRARY_DIR})
        file(COPY "${RGL_SRC_DIR}/include" DESTINATION ${RGL_PROJECT_DIR})
        file(WRITE ${RGL_STAMP_FILE} "${RGL_SOURCES_STAMP}")
    endif()
else() # Download RobotecGPULidar from the release
    message(WARNING "Downloading a stock RobotecGPULidar release: it lacks the colored-point-cloud "
                    "API required by this plugin version, so linking will fail. Use the default "
                    "RGL_BUILD_FROM_SOURCE=ON or provide RGL_CUSTOM_LIBRARY_PATH instead.")
    set(RGL_TAG "v0.21.0")
    set(RGL_API_CORE_HEADER_PATH "${RGL_API_HEADERS_DIR}/core.h")
    set(RGL_SO_ZIP_FILENAME "RGL-core-linux-x64.zip")
    set(RGL_SO_ZIP_URL "https://github.com/RobotecAI/RobotecGPULidar/releases/download/${RGL_TAG}/${RGL_SO_ZIP_FILENAME}")
    set(RGL_API_CORE_HEADER_URL "https://raw.githubusercontent.com/RobotecAI/RobotecGPULidar/${RGL_TAG}/include/rgl/api/core.h")

    # Download RGL library if not present
    if(NOT EXISTS ${RGL_SO_PATH})
        set(RGL_SO_ZIP_PATH "${RGL_LIBRARY_DIR}/${RGL_SO_ZIP_FILENAME}")
        file(DOWNLOAD ${RGL_SO_ZIP_URL} ${RGL_SO_ZIP_PATH})
        file(ARCHIVE_EXTRACT INPUT ${RGL_SO_ZIP_PATH}
                DESTINATION ${RGL_LIBRARY_DIR}
                PATTERNS ${RGL_SO_FILENAME}
                VERBOSE
        )
        file(REMOVE ${RGL_SO_ZIP_PATH})
    endif()

    # Download RGL API header if not present
    if(NOT EXISTS ${RGL_API_CORE_HEADER_PATH})
        file(DOWNLOAD ${RGL_API_CORE_HEADER_URL} ${RGL_API_CORE_HEADER_PATH})
    endif()
endif()

add_library(RobotecGPULidar SHARED IMPORTED GLOBAL)
set_target_properties(RobotecGPULidar PROPERTIES
    IMPORTED_LOCATION ${RGL_SO_PATH}
    INTERFACE_INCLUDE_DIRECTORIES ${RGL_INCLUDE_DIR}
)
