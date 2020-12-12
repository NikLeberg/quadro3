# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Enable colourized ninja output
add_compile_options(-fdiagnostics-color=always)

# Include the components directory of the main application:
#
set(EXTRA_COMPONENT_DIRS "../components")

# Set the components to include the tests for.
# This can be overriden from CMake cache:
# - when invoking CMake directly: cmake -D TEST_COMPONENTS="xxxxx"
#
# für i2c Test müssen i2c slave Pins mit Master Pins gebrückt werden, desshalb Test standartmässig deaktiviert
set(TEST_COMPONENTS "sensors" "bno")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(quadro2_test)
