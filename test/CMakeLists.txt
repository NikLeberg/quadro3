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
set(TEST_COMPONENTS "sensors" "bno")

# Set flag if we are run inside a CI build. As some tests can't be run without real hardware.
if(DEFINED ENV{CI})
    add_definitions(-DCI_TEST_IN_QEMU)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(quadro3_test)
