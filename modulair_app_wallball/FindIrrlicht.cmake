# - Find Irrlicht
# Find the Irrlicht includes and libraries
#
# Following variables are provided:
# IRRLICHT_FOUND
#     True if Irrlicht has been found
# IRRLICHT_DIR
#     Path to Irrlicht
# IRRLICHT_INCLUDE_DIRS
#     The include directories of Irrlicht
# IRRLICHT_LIBRARIES
#     Irrlicht library list
# IRRLICHT_VERSION

set(IRRLICHT_DIR "" CACHE PATH "Path to Irrlicht")

# Find include directory and library
find_path(IRRLICHT_INCLUDE_DIR NAMES irrlicht.h
    PATHS ${IRRLICHT_DIR}
        /Library/Frameworks/IrrFramework.framework/Versions/A/Headers/
        ${PROJECT_SOURCE_DIR}/dependencies/include/irrlicht
        ${PROJECT_SOURCE_DIR}/thirdparty/irrlicht
        ${PROJECT_SOURCE_DIR}/thirdparty/irrlicht-1.6.1
    PATH_SUFFIXES include irrlicht)

if(APPLE)
    find_library(IRRLICHT_LIBRARY NAMES IrrFramework PATHS /Library/Frameworks/IrrFramework.framework)
else()
    find_library(IRRLICHT_LIBRARY NAMES libIrrlicht Irrlicht
        PATHS ${IRRLICHT_DIR}/lib/Linux
            ${PROJECT_SOURCE_DIR}/dependencies/lib
            ${PROJECT_SOURCE_DIR}/thirdparty/irrlicht/lib/Linux
            ${PROJECT_SOURCE_DIR}/thirdparty/irrlicht-1.6.1/lib/Linux
            ${PROJECT_SOURCE_DIR})
endif()

# Determine Irrlicht version
if(EXISTS ${IRRLICHT_INCLUDE_DIR}/IrrCompileConfig.h)
    file(STRINGS ${IRRLICHT_INCLUDE_DIR}/IrrCompileConfig.h IRRLICHT_COMPILE_CONFIG REGEX IRRLICHT_VERSION)
    string(REGEX MATCH "IRRLICHT_VERSION_MAJOR ([0-9]+)" _tmp ${IRRLICHT_COMPILE_CONFIG})
    set(IRRLICHT_VERSION_MAJOR ${CMAKE_MATCH_1})
    string(REGEX MATCH "IRRLICHT_VERSION_MINOR ([0-9]+)" _tmp ${IRRLICHT_COMPILE_CONFIG})
    set(IRRLICHT_VERSION_MINOR ${CMAKE_MATCH_1})
    string(REGEX MATCH "IRRLICHT_VERSION_REVISION ([0-9]+)" _tmp ${IRRLICHT_COMPILE_CONFIG})
    set(IRRLICHT_VERSION_REVISION ${CMAKE_MATCH_1})
    set(IRRLICHT_VERSION "${IRRLICHT_VERSION_MAJOR}.${IRRLICHT_VERSION_MINOR}.${IRRLICHT_VERSION_REVISION}")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Irrlicht
    REQUIRED_VARS IRRLICHT_LIBRARY IRRLICHT_INCLUDE_DIR
    IRRLICHT_VERSION)

# Publish variables
set(IRRLICHT_INCLUDE_DIRS ${IRRLICHT_INCLUDE_DIR})
set(IRRLICHT_LIBRARIES ${IRRLICHT_LIBRARY})
mark_as_advanced(IRRLICHT_INCLUDE_DIR IRRLICHT_LIBRARY)

# Check if Xxf86vm is required when building for platforms using X11
if(UNIX AND NOT APPLE AND NOT CYGWIN)
    find_library(IRRLICHT_XF86VM_LIBRARY Xxf86vm)
    mark_as_advanced(IRRLICHT_XF86VM_LIBRARY)

    set(IRRLICHT_SNIPPET "#include <irrlicht.h>
        int main() { irr::createDevice(irr::video::EDT_NULL)\; return 0\; }")

    include(CheckCXXSourceCompiles)
    set(CMAKE_REQUIRED_INCLUDES ${IRRLICHT_INCLUDE_DIR})
    set(CMAKE_REQUIRED_LIBRARIES ${IRRLICHT_LIBRARIES})
    check_cxx_source_compiles(${IRRLICHT_SNIPPET} IRRLICHT_WITHOUT_XF86VM)

    # If it did not work without Xxf86vm library try with it again
    if(NOT IRRLICHT_WITHOUT_XF86VM)
        set(CMAKE_REQUIRED_LIBRARIES ${IRRLICHT_LIBRARIES} ${IRRLICHT_XF86VM_LIBRARY})
        check_cxx_source_compiles(${IRRLICHT_SNIPPET} IRRLICHT_WITH_XF86VM)

        if(NOT IRRLICHT_WITH_XF86VM)
            message(WARNING "Irrlicht does not compile with and without Xxf86vm")
        endif()

        # Add Xxf86vm nevertheless as tests might fail under strange circumstances
        set(IRRLICHT_LIBRARIES ${IRRLICHT_LIBRARIES} ${IRRLICHT_XF86VM_LIBRARY})
    endif()
endif()
