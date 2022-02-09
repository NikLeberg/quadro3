# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Enable colourized ninja output
add_compile_options(-fdiagnostics-color=always)

# Include the components directory of the main application:
#
set(EXTRA_COMPONENT_DIRS "../components")

# Set the components to include the tests for.
# for i2c tests we need a testbed with pins wired together, disable tests for now
set(TEST_COMPONENTS "adder" "sensors" "config" "bno")

# If we are being run inside the CI system
if(DEFINED ENV{CI})
    set(TEST_IN_QEMU "1")
endif()

# If we are instructed to run tests inside QEMU
if(DEFINED TEST_IN_QEMU AND TEST_IN_QEMU STREQUAL "1")
    # add precompiler definition so code can adapt and only run QEMU enabled tests
    add_definitions(-DTEST_IN_QEMU)
    # espressif qemu does not support fpu hardware acceleration, use double
    add_definitions(-DUNITY_FLOAT_TYPE=double)
    # add folder of mocked components
    list(APPEND EXTRA_COMPONENT_DIRS "./mocks")
endif()

# Get the cmake target of the currently processed component
# - sets COMPONENT_REAL_TARGET to the real target name
function(get_current_component_target)
    get_filename_component(DIR ${CMAKE_CURRENT_LIST_DIR} NAME)
    if(DIR STREQUAL "test")
        get_filename_component(PARENT_DIR ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)
        get_filename_component(PARENT_COMPONENT_NAME ${PARENT_DIR} NAME)
        set(COMPONENT_ALIAS_TARGET "${PARENT_COMPONENT_NAME}::test")
    else()
        set(COMPONENT_ALIAS_TARGET ${DIR})
    endif()
    idf_component_get_property(COMPONENT_REAL_TARGET ${COMPONENT_ALIAS_TARGET} COMPONENT_LIB)
    set(COMPONENT_REAL_TARGET ${COMPONENT_REAL_TARGET} PARENT_SCOPE)
endfunction()

# Append a property to the currently processed component
function(append_to_current_component_property prop value)
    get_current_component_target()
    set_property(TARGET ${COMPONENT_REAL_TARGET} APPEND PROPERTY ${prop} ${value})
endfunction()

# Allow wraping of functions
function(set_wrapped_functions_of_current_component funcs)
    foreach(func IN LISTS funcs)
        append_to_current_component_property(INTERFACE_LINK_LIBRARIES "-Wl,--wrap=${func}")
    endforeach()
endfunction()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(quadro3_test)
