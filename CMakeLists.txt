cmake_minimum_required(VERSION 3.16)
project(gz_dynamic_terrain_plugin VERSION 0.2.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_SIMULATOR "Build the selected simulator adapter" ON)
option(BUILD_GUI "Build the modern Gazebo terrain preview GUI plugin" ON)
option(BUILD_RENDER_TESTS "Run graphics smoke tests (requires a display or Xvfb)" OFF)
set(GZ_DISTRO "harmonic" CACHE STRING "Gazebo release: harmonic or jetty")
set_property(CACHE GZ_DISTRO PROPERTY STRINGS harmonic jetty)
if(NOT GZ_DISTRO MATCHES "^(harmonic|jetty)$")
    message(FATAL_ERROR "Unknown GZ_DISTRO='${GZ_DISTRO}'. Choose harmonic or jetty.")
endif()
if(DEFINED DYNAMIC_TERRAIN_CONFIGURED_DISTRO AND
   NOT DYNAMIC_TERRAIN_CONFIGURED_DISTRO STREQUAL GZ_DISTRO)
    message(FATAL_ERROR "GZ_DISTRO changed in an existing build directory. Use a fresh build-${GZ_DISTRO} directory.")
endif()
set(DYNAMIC_TERRAIN_CONFIGURED_DISTRO "${GZ_DISTRO}" CACHE INTERNAL "Original build backend")

include(GNUInstallDirs)
include(CTest)
find_package(Threads REQUIRED)
find_package(CURL REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

add_library(gz-dynamic-terrain-core SHARED
    src/core/TerrainTypes.cc
    src/core/TileStore.cc
    src/core/TerrainBuilder.cc
    src/core/TerrainRuntime.cc
    src/core/CollisionTerrain.cc
)
target_include_directories(gz-dynamic-terrain-core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/dynamic_terrain/core>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(gz-dynamic-terrain-core
    PUBLIC Threads::Threads ${OpenCV_LIBS}
    PRIVATE CURL::libcurl
)
target_compile_options(gz-dynamic-terrain-core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(gz-dynamic-terrain-core PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
set(terrain_install_targets gz-dynamic-terrain-core)

if(BUILD_SIMULATOR)
    include(cmake/GazeboBackend.cmake)
    find_package(Protobuf REQUIRED)
    protobuf_generate_cpp(GUI_PROTO_SOURCES GUI_PROTO_HEADERS
        src/adapters/gzsim/gui/GuiTerrain.proto)

    add_library(dynamic-terrain-gzsim-adapter STATIC
        src/adapters/SdfConfig.cc
        src/adapters/CollisionSdf.cc
        src/adapters/gzsim/GzTerrainRenderer.cc
        src/adapters/gzsim/Ogre2ResourceCleanup.cc
        src/adapters/gzsim/ModelConfigRegistry.cc
        src/adapters/gzsim/gui/GuiTerrain.cc
        ${GUI_PROTO_SOURCES}
    )
    add_library(dynamic-terrain-gzsim-api INTERFACE)
    target_include_directories(dynamic-terrain-gzsim-api INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/dynamic_terrain/adapters
        ${CMAKE_CURRENT_SOURCE_DIR}/include/dynamic_terrain/adapters/gzsim
        ${CMAKE_CURRENT_SOURCE_DIR}/src/adapters/gzsim
        ${CMAKE_CURRENT_SOURCE_DIR}/src/adapters/gzsim/gui
        ${CMAKE_CURRENT_BINARY_DIR}
    )
    target_link_libraries(dynamic-terrain-gzsim-api INTERFACE
        gz-dynamic-terrain-core
        ${DYNAMIC_TERRAIN_SIM_TARGET}
        ${DYNAMIC_TERRAIN_RENDER_TARGET}
        GzOGRE2::GzOGRE2
        protobuf::libprotobuf
    )
    target_compile_definitions(dynamic-terrain-gzsim-api INTERFACE
        ${DYNAMIC_TERRAIN_BACKEND_DEFINITION})
    target_link_libraries(dynamic-terrain-gzsim-adapter PUBLIC dynamic-terrain-gzsim-api)

    add_library(gz-dynamic-terrain-system SHARED
        src/adapters/gzsim/DynamicTerrainSystem.cc)
    target_link_libraries(gz-dynamic-terrain-system PRIVATE
        dynamic-terrain-gzsim-adapter ${DYNAMIC_TERRAIN_PLUGIN_TARGET})
    set_target_properties(gz-dynamic-terrain-system PROPERTIES
        OUTPUT_NAME "gz-dynamic-terrain-system"
        INSTALL_RPATH "$ORIGIN"
        INSTALL_RPATH_USE_LINK_PATH TRUE)
    list(APPEND terrain_install_targets gz-dynamic-terrain-system)

    if(BUILD_GUI)
        add_library(DynamicTerrainGui SHARED
            src/adapters/gzsim/gui/DynamicTerrainGui.cc
            src/adapters/gzsim/gui/DynamicTerrainGui.qrc)
        target_link_libraries(DynamicTerrainGui PRIVATE
            gz-dynamic-terrain-system dynamic-terrain-gzsim-api
            ${DYNAMIC_TERRAIN_GUI_TARGET}
            ${DYNAMIC_TERRAIN_QT_TARGET})
        set_target_properties(DynamicTerrainGui PROPERTIES
            AUTORCC ON INSTALL_RPATH "$ORIGIN"
            INSTALL_RPATH_USE_LINK_PATH TRUE)
        list(APPEND terrain_install_targets DynamicTerrainGui)
    endif()

    if(UNIX AND NOT APPLE)
        target_link_options(gz-dynamic-terrain-system PRIVATE "LINKER:-Bsymbolic-functions")
        if(BUILD_GUI)
            target_link_options(DynamicTerrainGui PRIVATE "LINKER:-Bsymbolic-functions")
        endif()
    endif()
endif()

if(BUILD_TESTING)
    file(GLOB terrain_core_tests CONFIGURE_DEPENDS tests/core/*_TEST.cc)
    foreach(test_source IN LISTS terrain_core_tests)
        get_filename_component(test_name "${test_source}" NAME_WE)
        add_executable(${test_name} "${test_source}")
        target_link_libraries(${test_name} PRIVATE gz-dynamic-terrain-core)
        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
    add_executable(CollisionSdf_TEST tests/CollisionSdf_TEST.cc src/adapters/CollisionSdf.cc)
    target_link_libraries(CollisionSdf_TEST PRIVATE gz-dynamic-terrain-core)
    add_test(NAME CollisionSdf_TEST COMMAND CollisionSdf_TEST)
    add_test(NAME CheckPluginFilenames COMMAND ${CMAKE_COMMAND}
        -DEXAMPLES_DIR=${CMAKE_CURRENT_SOURCE_DIR}/examples
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/CheckPluginFilenames.cmake)
    add_test(NAME CheckCoreBoundary COMMAND ${CMAKE_COMMAND}
        -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/CheckCoreBoundary.cmake)
    if(BUILD_SIMULATOR)
        foreach(test_name GuiTerrain_TEST GzGeographicTransform_TEST)
            add_executable(${test_name} tests/gzsim/${test_name}.cc)
            target_link_libraries(${test_name} PRIVATE gz-dynamic-terrain-system dynamic-terrain-gzsim-api)
            add_test(NAME ${test_name} COMMAND ${test_name})
        endforeach()
        add_executable(SystemPluginSmoke_TEST tests/gzsim/SystemPluginSmoke_TEST.cc)
        target_link_libraries(SystemPluginSmoke_TEST PRIVATE gz-dynamic-terrain-system dynamic-terrain-gzsim-api
            ${DYNAMIC_TERRAIN_PLUGIN_LOADER_TARGET})
        target_compile_definitions(SystemPluginSmoke_TEST PRIVATE
            TERRAIN_SYSTEM_LIBRARY="$<TARGET_FILE:gz-dynamic-terrain-system>")
        add_dependencies(SystemPluginSmoke_TEST gz-dynamic-terrain-system)
        add_test(NAME SystemPluginSmoke_TEST COMMAND SystemPluginSmoke_TEST)
        set_tests_properties(SystemPluginSmoke_TEST GuiTerrain_TEST PROPERTIES TIMEOUT 60)
        add_executable(Ogre2TextureLifecycle_TEST tests/Ogre2TextureLifecycle_TEST.cc)
        target_link_libraries(Ogre2TextureLifecycle_TEST PRIVATE gz-dynamic-terrain-system dynamic-terrain-gzsim-api)
        target_compile_definitions(Ogre2TextureLifecycle_TEST PRIVATE
            TERRAIN_RENDER_ENGINE_LIBRARY="$<TARGET_FILE:${DYNAMIC_TERRAIN_RENDER_LIBRARY_TARGET}>")
        if(BUILD_RENDER_TESTS)
            add_test(NAME Ogre2TextureLifecycle_TEST COMMAND Ogre2TextureLifecycle_TEST)
            set_tests_properties(Ogre2TextureLifecycle_TEST PROPERTIES TIMEOUT 60)
        endif()
        if(BUILD_GUI)
            add_executable(GuiPlugin_TEST tests/gzsim/GuiPlugin_TEST.cc)
            target_link_libraries(GuiPlugin_TEST PRIVATE gz-dynamic-terrain-system dynamic-terrain-gzsim-api
                ${DYNAMIC_TERRAIN_GUI_TARGET} ${DYNAMIC_TERRAIN_QT_TARGET})
            target_compile_definitions(GuiPlugin_TEST PRIVATE
                TERRAIN_RENDER_ENGINE_LIBRARY="$<TARGET_FILE:${DYNAMIC_TERRAIN_RENDER_LIBRARY_TARGET}>")
            add_dependencies(GuiPlugin_TEST DynamicTerrainGui)
            if(BUILD_RENDER_TESTS)
                add_test(NAME GuiPlugin_TEST COMMAND GuiPlugin_TEST
                    $<TARGET_FILE_DIR:DynamicTerrainGui>)
                set_tests_properties(GuiPlugin_TEST PROPERTIES TIMEOUT 60)
            endif()
        endif()
    endif()
endif()

install(TARGETS ${terrain_install_targets}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/dynamic_terrain/core
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dynamic_terrain)
