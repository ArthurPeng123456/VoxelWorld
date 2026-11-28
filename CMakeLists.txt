cmake_minimum_required(VERSION 3.16)

if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

project(VoxelWorld LANGUAGES CXX)

# Bullet's ABI and every translation unit must agree on btScalar precision.
add_compile_definitions(BT_USE_DOUBLE_PRECISION)

option(VOXEL_SYSTEM_BULLET "Use an installed Bullet SDK instead of the pinned Bullet3 source" OFF)
set(VOXEL_BULLET_ARCHIVE "" CACHE FILEPATH "Optional local copy of the pinned Bullet3 archive")
if(VOXEL_SYSTEM_BULLET)
    find_package(Bullet REQUIRED)
else()
    include(FetchContent)
    if(POLICY CMP0135)
        cmake_policy(SET CMP0135 NEW)
    endif()
    set(bullet_archive https://deb.debian.org/debian/pool/main/b/bullet/bullet_3.24+dfsg.orig.tar.xz)
    if(VOXEL_BULLET_ARCHIVE)
        set(bullet_archive "${VOXEL_BULLET_ARCHIVE}")
    endif()
    FetchContent_Declare(bullet
        URL "${bullet_archive}"
        URL_HASH SHA256=43c1edae2024ba18bb5d7bfd3bb2394b066a79151a29789504a2d8efcdc333d7
        SOURCE_SUBDIR unused TIMEOUT 120)
    if(CMAKE_VERSION VERSION_LESS 3.18)
        FetchContent_GetProperties(bullet)
        if(NOT bullet_POPULATED)
            FetchContent_Populate(bullet)
        endif()
    else()
        FetchContent_MakeAvailable(bullet)
    endif()
    set(BULLET_PHYSICS_SOURCE_DIR "${bullet_SOURCE_DIR}")
    set(BULLET_VERSION 3.24)
    add_subdirectory("${bullet_SOURCE_DIR}/src/LinearMath" "${bullet_BINARY_DIR}/LinearMath")
    add_subdirectory("${bullet_SOURCE_DIR}/src/BulletCollision" "${bullet_BINARY_DIR}/BulletCollision")
    add_subdirectory("${bullet_SOURCE_DIR}/src/BulletDynamics" "${bullet_BINARY_DIR}/BulletDynamics")
    set(BULLET_INCLUDE_DIRS "${bullet_SOURCE_DIR}/src")
    set(BULLET_LIBRARIES BulletDynamics BulletCollision LinearMath)
endif()
# ========== oneTBB：与参考项目同一套本地安装 ==========
set(TBB_ROOT "D:/software/tbb/tbb/2023.2" CACHE PATH "oneTBB 安装根目录")
if(NOT TBB_DIR)
    set(TBB_DIR "${TBB_ROOT}/lib/cmake/tbb" CACHE PATH "TBB CMake package directory" FORCE)
endif()
find_package(TBB REQUIRED)
find_package(Threads REQUIRED)

# 禁止 vcpkg 对 OpenVDB 自动查找依赖
set(VCPKG_HOOK_OPENPACKAGE_OpenVDB OFF CACHE INTERNAL "")
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)

# ========== 自动选择 Debug / Release OpenVDB 路径 ==========
set(VOXEL_OPENVDB_INSTALL_ROOT "D:/UE/openvdb-14.0.0/out/install" CACHE PATH "OpenVDB installation root containing x64-Debug and x64-Release")
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(_voxel_openvdb_default_root "${VOXEL_OPENVDB_INSTALL_ROOT}/x64-Debug")
else()
    set(_voxel_openvdb_default_root "${VOXEL_OPENVDB_INSTALL_ROOT}/x64-Release")
endif()
set(VOXEL_OPENVDB_ROOT "${_voxel_openvdb_default_root}" CACHE PATH "OpenVDB installation prefix")
set(OPENVDB_ROOT "${VOXEL_OPENVDB_ROOT}")

if(NOT EXISTS "${OPENVDB_ROOT}/lib/cmake/OpenVDB/OpenVDBConfig.cmake")
    message(FATAL_ERROR
        "OpenVDB CMake package not found: ${OPENVDB_ROOT}/lib/cmake/OpenVDB/OpenVDBConfig.cmake\n"
        "Build and install OpenVDB, or set -DOPENVDB_ROOT=<install-prefix> / -DVOXEL_OPENVDB_INSTALL_ROOT=<root>.")
endif()

list(PREPEND CMAKE_PREFIX_PATH "${OPENVDB_ROOT}")
find_package(OpenVDB CONFIG REQUIRED)

set(VOXEL_OPENVDB_DEBUG_LIBRARY "" CACHE FILEPATH "Matching MSVC Debug library for the bundled OpenVDB")

if(MSVC AND VOXEL_OPENVDB_DEBUG_LIBRARY)
    # OpenVDB's Find module exposes only one library location on Windows.
    get_target_property(openvdb_release OpenVDB::openvdb IMPORTED_LOCATION)
    set_target_properties(OpenVDB::openvdb PROPERTIES
        IMPORTED_CONFIGURATIONS "DEBUG;RELEASE"
        IMPORTED_LOCATION_DEBUG "${VOXEL_OPENVDB_DEBUG_LIBRARY}"
        IMPORTED_LOCATION_RELEASE "${openvdb_release}")
endif()

function(voxel_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

add_library(voxel_core STATIC src/terrain.cpp src/planet_geology.cpp src/lod.cpp src/storage.cpp src/collision.cpp src/world.cpp)
target_compile_features(voxel_core PUBLIC cxx_std_17)
if(MSVC)
    target_include_directories(voxel_core PRIVATE ${BULLET_INCLUDE_DIRS})
else()
    target_include_directories(voxel_core SYSTEM PRIVATE ${BULLET_INCLUDE_DIRS})
endif()
target_link_libraries(voxel_core PRIVATE ${BULLET_LIBRARIES} OpenVDB::openvdb TBB::tbb Threads::Threads)
voxel_warnings(voxel_core)

add_executable(voxel_world src/main.cpp)
target_link_libraries(voxel_world PRIVATE voxel_core)
voxel_warnings(voxel_world)
install(TARGETS voxel_world RUNTIME DESTINATION .)

if(WIN32)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/shaders/df64.hlsli" VOXEL_PRECISION_SHADER)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/shaders/atmosphere.hlsl" VOXEL_ATMOSPHERE_SHADER)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/shaders/terrain_material.hlsli" VOXEL_TERRAIN_MATERIAL_SHADER)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/shaders/deferred.hlsl" VOXEL_DEFERRED_SHADER)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/shaders/clouds.hlsl" VOXEL_CLOUD_SHADER)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/shaders/temporal_common.hlsli" VOXEL_TEMPORAL_COMMON_SHADER)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/shaders/temporal_resolve.hlsl" VOXEL_TEMPORAL_RESOLVE_SHADER)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS src/shaders/temporal_common.hlsli src/shaders/temporal_resolve.hlsl)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS src/shaders/clouds.hlsl)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS src/shaders/deferred.hlsl src/shaders/terrain_material.hlsli)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS src/shaders/df64.hlsli src/shaders/atmosphere.hlsl)
    configure_file(src/shaders.hpp.in generated/shaders.hpp @ONLY)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/tests/gpu_precision.hlsl" VOXEL_TEST_SHADER)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS tests/gpu_precision.hlsl)
    configure_file(tests/precision_test_shader.hpp.in generated/precision_test_shader.hpp @ONLY)
    add_executable(voxel_gpu_precision tests/gpu_precision.cpp src/cloud_noise.cpp)
    target_compile_features(voxel_gpu_precision PRIVATE cxx_std_17)
    target_compile_definitions(voxel_gpu_precision PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
    target_include_directories(voxel_gpu_precision PRIVATE src "${CMAKE_CURRENT_BINARY_DIR}/generated")
    target_link_libraries(voxel_gpu_precision PRIVATE d3d11 d3dcompiler)
    voxel_warnings(voxel_gpu_precision)
    target_compile_definitions(voxel_core PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
    add_executable(VoxelWorldViewer WIN32 src/viewer.cpp src/dx12_renderer.cpp src/cloud_noise.cpp src/lod_range_dialog.cpp src/render_settings_dialog.cpp src/invoker_settings_dialog.cpp)
    target_include_directories(VoxelWorldViewer PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
    target_compile_definitions(VoxelWorldViewer PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
    target_link_libraries(VoxelWorldViewer PRIVATE voxel_core TBB::tbb d3d12 dxgi d3dcompiler gdi32 user32 shell32 psapi comctl32)
    if(MINGW)
        target_link_options(VoxelWorldViewer PRIVATE -municode)
    endif()
    voxel_warnings(VoxelWorldViewer)
    set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT VoxelWorldViewer)
    install(TARGETS VoxelWorldViewer RUNTIME DESTINATION .)
    if(MSVC)
        install(FILES "$<TARGET_PDB_FILE:voxel_world>" "$<TARGET_PDB_FILE:VoxelWorldViewer>"
            DESTINATION . CONFIGURATIONS Debug)
    endif()
    install(FILES third_party/VTK-Copyright.txt DESTINATION licenses)
    if(NOT VOXEL_SYSTEM_BULLET)
        install(FILES "${bullet_SOURCE_DIR}/LICENSE.txt" DESTINATION licenses RENAME Bullet-LICENSE.txt)
    endif()
endif()

enable_testing()
add_executable(voxel_clipmap_distribution tests/clipmap_distribution.cpp)
target_include_directories(voxel_clipmap_distribution PRIVATE src)
target_link_libraries(voxel_clipmap_distribution PRIVATE voxel_core)
voxel_warnings(voxel_clipmap_distribution)
add_test(NAME voxel_world_correctness COMMAND voxel_world --self-test)
if(CMAKE_CONFIGURATION_TYPES)
    add_test(NAME voxel_clipmap_distribution COMMAND voxel_clipmap_distribution 16 64
        CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
    add_test(NAME voxel_world_global_terrain COMMAND voxel_world --self-test-global
        CONFIGURATIONS Release RelWithDebInfo MinSizeRel)
elseif(CMAKE_BUILD_TYPE MATCHES "^(Release|RelWithDebInfo|MinSizeRel)$")
    add_test(NAME voxel_clipmap_distribution COMMAND voxel_clipmap_distribution 16 64)
    add_test(NAME voxel_world_global_terrain COMMAND voxel_world --self-test-global)
endif()
if(WIN32)
    add_test(NAME voxel_gpu_precision COMMAND voxel_gpu_precision)
endif()

# ========== 复制当前配置对应的 OpenVDB dll ==========
set(OPENVDB_BIN "${OPENVDB_ROOT}/bin")
file(GLOB OPENVDB_DLLS "${OPENVDB_BIN}/*.dll")

add_custom_command(TARGET voxel_world POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${OPENVDB_DLLS}
    $<TARGET_FILE_DIR:voxel_world>
)
if(WIN32)
    add_custom_command(TARGET VoxelWorldViewer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${OPENVDB_DLLS}
        $<TARGET_FILE_DIR:VoxelWorldViewer>
    )
endif()

# ========== oneTBB DLL 自动拷贝（根目录与参考项目一致）==========
set(TBB_OFFLINE_ROOT "${TBB_ROOT}" CACHE PATH "oneTBB 安装根目录（留空则自动探测）")

set(_tbb_candidate_roots "")
if(TBB_OFFLINE_ROOT)
    list(APPEND _tbb_candidate_roots "${TBB_OFFLINE_ROOT}")
endif()
if(TBB_DIR)
    # TBB_DIR 类似 <root>/lib/cmake/tbb，向上三层即安装根
    get_filename_component(_tbb_root_from_config "${TBB_DIR}/../../.." ABSOLUTE)
    list(APPEND _tbb_candidate_roots "${_tbb_root_from_config}")
endif()
if(DEFINED ENV{ONEAPI_ROOT})
    list(APPEND _tbb_candidate_roots "$ENV{ONEAPI_ROOT}/tbb/latest")
endif()
list(APPEND _tbb_candidate_roots "C:/Program Files (x86)/Intel/oneAPI/tbb/latest")

set(TBB_DLL_DEBUG "")
set(TBB_DLL_RELEASE "")
foreach(_root IN LISTS _tbb_candidate_roots)
    if(_root AND IS_DIRECTORY "${_root}")
        # 离线版 dll 可能位于 bin/ 或 redist/intel64/vc14/
        foreach(_sub "bin" "redist/intel64/vc14" "bin/intel64/vc14")
            if(NOT TBB_DLL_DEBUG AND EXISTS "${_root}/${_sub}/tbb12_debug.dll")
                set(TBB_DLL_DEBUG "${_root}/${_sub}/tbb12_debug.dll")
            endif()
            if(NOT TBB_DLL_RELEASE AND EXISTS "${_root}/${_sub}/tbb12.dll")
                set(TBB_DLL_RELEASE "${_root}/${_sub}/tbb12.dll")
            endif()
        endforeach()
    endif()
endforeach()

if(NOT TBB_DLL_DEBUG AND NOT TBB_DLL_RELEASE)
    message(WARNING
        "未找到 oneTBB 的 tbb12.dll / tbb12_debug.dll，跳过 DLL 拷贝。"
        "若运行时报缺少 tbb12*.dll，请用 -DTBB_OFFLINE_ROOT=<oneTBB 安装根目录> 指定。")
else()
    if(NOT TBB_DLL_DEBUG)
        set(TBB_DLL_DEBUG "${TBB_DLL_RELEASE}")
    endif()
    if(NOT TBB_DLL_RELEASE)
        set(TBB_DLL_RELEASE "${TBB_DLL_DEBUG}")
    endif()
    message(STATUS "oneTBB DLL: ${TBB_DLL_DEBUG} / ${TBB_DLL_RELEASE}")
    set(TBB_DLL "$<IF:$<CONFIG:Debug>,${TBB_DLL_DEBUG},${TBB_DLL_RELEASE}>")

    add_custom_command(TARGET voxel_world POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${TBB_DLL}" "$<TARGET_FILE_DIR:voxel_world>/"
        COMMENT "Copy oneTBB dll..."
    )
    if(WIN32)
        add_custom_command(TARGET VoxelWorldViewer POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different "${TBB_DLL}" "$<TARGET_FILE_DIR:VoxelWorldViewer>/"
            COMMENT "Copy oneTBB dll for viewer..."
        )
    endif()
endif()
