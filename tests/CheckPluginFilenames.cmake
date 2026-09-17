file(GLOB_RECURSE terrain_example_files
    "${EXAMPLES_DIR}/*.sdf"
    "${EXAMPLES_DIR}/*.config"
    "${EXAMPLES_DIR}/*.world")

set(found_plugin FALSE)
foreach(example_file IN LISTS terrain_example_files)
    file(READ "${example_file}" contents)
    if(contents MATCHES "libgz-dynamic-terrain-system-v[0-9_]+\\.so")
        message(FATAL_ERROR
            "${example_file} still references a stale versioned terrain plugin")
    endif()
    if(contents MATCHES "libgz-dynamic-terrain-system\\.so")
        set(found_plugin TRUE)
    endif()
endforeach()

if(NOT found_plugin)
    message(FATAL_ERROR
        "No example references libgz-dynamic-terrain-system.so")
endif()

foreach(distro harmonic jetty)
    set(snippet "${EXAMPLES_DIR}/${distro}/plugin_snippet.sdf")
    if(NOT EXISTS "${snippet}")
        message(FATAL_ERROR "Missing ${distro} example")
    endif()
    file(READ "${snippet}" contents)
    if(NOT contents MATCHES "custom::DynamicTerrainConfig" OR
       NOT contents MATCHES "libgz-dynamic-terrain-system\\.so")
        message(FATAL_ERROR "${snippet} does not preserve the modern plugin contract")
    endif()
endforeach()
