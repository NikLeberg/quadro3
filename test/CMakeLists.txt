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
    # always set flag to run tests in QEMU
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
    # Custom target that prepares and runs tests in QEMU. QEMU is configured to
    # not restart, so that a call to esp_restart() or other reset reasons e.g.
    # core panic stops QEMU. Afterwards a scrit is ran that checks if the run
    # was successfull or not.
    add_custom_target(qemu_run
        COMMAND esptool.py --chip esp32 merge_bin --fill-flash-size 4MB -o qemu_image.bin @flash_args
        COMMAND qemu-system-xtensa -nographic -machine esp32 -drive file=qemu_image.bin,if=mtd,format=raw -no-reboot -d guest_errors,unimp | tee qemu_log.txt
        COMMAND ../qemu_check.sh
        BYPRODUCTS qemu_image.bin qemu_log.txt
        WORKING_DIRECTORY ${CMAKE_CURRENT_BUILD_DIR}
        COMMENT "Running qemu..."
        DEPENDS gen_project_binary bootloader
        VERBATIM
        USES_TERMINAL
    )
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
