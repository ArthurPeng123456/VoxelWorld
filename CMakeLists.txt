cmake_minimum_required(VERSION 3.16)
project(VoxelWorld LANGUAGES CXX)

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
# 禁止vcpkg对OpenVDB自动查找依赖
set(VCPKG_HOOK_OPENPACKAGE_OpenVDB OFF CACHE INTERNAL "")
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)

# ========== 自动选择 Debug / Release OpenVDB 路径 ==========
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(OPENVDB_ROOT "D:/UE/openvdb-14.0.0/out/install/x64-Debug")
else()
    set(OPENVDB_ROOT "D:/UE/openvdb-14.0.0/out/install/x64-Release")
endif()
set(CMAKE_PREFIX_PATH "${OPENVDB_ROOT};${CMAKE_PREFIX_PATH}")
find_package(OpenVDB CONFIG REQUIRED)
# ==========================================================

function(voxel_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

add_library(voxel_core STATIC src/terrain.cpp src/storage.cpp src/collision.cpp src/world.cpp)
target_compile_features(voxel_core PUBLIC cxx_std_17)
if(MSVC)
    target_include_directories(voxel_core PRIVATE ${BULLET_INCLUDE_DIRS})
else()
    target_include_directories(voxel_core SYSTEM PRIVATE ${BULLET_INCLUDE_DIRS})
endif()
# 移除 TBB::tbb
target_link_libraries(voxel_core PRIVATE ${BULLET_LIBRARIES} OpenVDB::openvdb)
voxel_warnings(voxel_core)

add_executable(voxel_world src/main.cpp)
target_link_libraries(voxel_world PRIVATE voxel_core)
voxel_warnings(voxel_world)

install(TARGETS voxel_world RUNTIME DESTINATION .)

if(WIN32)
    target_compile_definitions(voxel_core PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
    add_executable(VoxelWorldViewer WIN32 src/viewer.cpp src/dx12_renderer.cpp)
    target_compile_definitions(VoxelWorldViewer PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
    target_link_libraries(VoxelWorldViewer PRIVATE voxel_core d3d12 dxgi d3dcompiler gdi32 user32 shell32)
    if(MINGW)
        target_link_options(VoxelWorldViewer PRIVATE -municode)
    endif()
    voxel_warnings(VoxelWorldViewer)
    install(TARGETS VoxelWorldViewer RUNTIME DESTINATION .)
    install(FILES third_party/VTK-Copyright.txt DESTINATION licenses)
    if(NOT VOXEL_SYSTEM_BULLET)
        install(FILES "${bullet_SOURCE_DIR}/LICENSE.txt" DESTINATION licenses RENAME Bullet-LICENSE.txt)
    endif()
endif()

enable_testing()
add_test(NAME voxel_world_correctness COMMAND voxel_world --self-test)

# 复制当前配置对应的 OpenVDB dll（已经删掉 TBB_DLLS）
set(OPENVDB_BIN "${OPENVDB_ROOT}/bin")
file(GLOB OPENVDB_DLLS "${OPENVDB_BIN}/*.dll")

add_custom_command(TARGET voxel_world POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${OPENVDB_DLLS}
    $<TARGET_FILE_DIR:voxel_world>
)

add_custom_command(TARGET VoxelWorldViewer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${OPENVDB_DLLS}
    $<TARGET_FILE_DIR:VoxelWorldViewer>
)

# ========== Intel oneAPI 离线安装版 oneTBB 自动拷贝DLL ==========
set(TBB_OFFLINE_ROOT "C:/Program Files (x86)/Intel/oneAPI/tbb/latest" CACHE PATH "oneTBB离线安装根目录")

# 离线版dll固定在 redist/intel64/vc14/
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(TBB_DLL "${TBB_OFFLINE_ROOT}/bin/tbb12_debug.dll")
else()
    set(TBB_DLL "${TBB_OFFLINE_ROOT}/bin/tbb12.dll")
endif()

# 拷贝TBB dll
add_custom_command(TARGET voxel_world POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${TBB_DLL}" $<TARGET_FILE_DIR:voxel_world>/
    COMMENT "【离线oneTBB】Copy tbb dll..."
)

# 顺带拷贝OpenVDB dll（Debug名字是openvdb_d.dll）
set(OPENVDB_DLL "${OPENVDB_ROOT}/bin/$<IF:$<CONFIG:Debug>,openvdb.dll,openvdb.dll>")
add_custom_command(TARGET voxel_world POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OPENVDB_DLL}" $<TARGET_FILE_DIR:voxel_world>/
    COMMENT "Copy OpenVDB dll..."
)
