if(WIN32)
    cmake_minimum_required(VERSION 3.4)
else()
    cmake_minimum_required(VERSION 3.0)
endif()

project(iCubProprioception
        LANGUAGES CXX
        VERSION 0.3.1.0)

include(GNUInstallDirs)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

### iCub-contrib project
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()
icubcontrib_add_uninstall_target()

### Options
# Install SIM applications?
option(INSTALL_SIM_APP "Install yarpmanager applications for the iCub_SIM" OFF)

# Build test related commands?
option(BUILD_TESTING "Create tests using CMake" OFF)
if(BUILD_TESTING)
    enable_testing()
endif()

# Support RPATH?
option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
                          DEPENDS ENABLE_RPATH
                          USE_LINK_PATH)

### Compile- and install-related commands
add_subdirectory(src)

# Add integration tests (unit tests for each library should be in each sublibrary directory)
if(BUILD_TESTING)
    add_subdirectory(test)
endif()
