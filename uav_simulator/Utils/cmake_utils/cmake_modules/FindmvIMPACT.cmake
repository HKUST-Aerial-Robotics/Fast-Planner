macro(mvIMPACT_REPORT_NOT_FOUND REASON_MSG)
    unset(mvIMPACT_FOUND)
    unset(mvIMPACT_INCLUDE_DIRS)
    unset(mvIMPACT_LIBRARIES)
    unset(mvIMPACT_WORLD_VERSION)
    unset(mvIMPACT_MAJOR_VERSION)
    unset(mvIMPACT_MINOR_VERSION)
    # Make results of search visible in the CMake GUI if mvimpact has not
    # been found so that user does not have to toggle to advanced view.
    mark_as_advanced(CLEAR mvIMPACT_INCLUDE_DIR)
    # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
    # use the camelcase library name, not uppercase.
    if(Mvimpact_FIND_QUIETLY)
        message(STATUS "Failed to find mvimpact - " ${REASON_MSG} ${ARGN})
    elseif(Mvimpact_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find mvimpact - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find mvimpact - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(mvIMPACT_REPORT_NOT_FOUND)

# Search user-installed locations first, so that we prefer user installs
# to system installs where both exist.
get_filename_component(BLUEFOX2_DIR ${CMAKE_CURRENT_SOURCE_DIR} REALPATH)
list(APPEND mvIMPACT_CHECK_INCLUDE_DIRS
    /opt/mvIMPACT_acquire
    /opt/mvIMPACT_Acquire
    )
execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCH)

if(NOT ARCH)
set(ARCH "arm64")
elseif(${ARCH} STREQUAL "aarch64")
set(ARCH "arm64")
endif()

list(APPEND mvIMPACT_CHECK_LIBRARY_DIRS
    /opt/mvIMPACT_acquire/lib/${ARCH}
    /opt/mvIMPACT_Acquire/lib/${ARCH}
    )

# Check general hints
if(mvIMPACT_HINTS AND EXISTS ${mvIMPACT_HINTS})
    set(mvIMPACT_INCLUDE_DIR_HINTS ${mvIMPACT_HINTS}/include)
    set(mvIMPACT_LIBRARY_DIR_HINTS ${mvIMPACT_HINTS}/lib)
endif()

# Search supplied hint directories first if supplied.
# Find include directory for mvimpact
find_path(mvIMPACT_INCLUDE_DIR
    NAMES mvIMPACT_CPP/mvIMPACT_acquire.h
    PATHS ${mvIMPACT_INCLUDE_DIR_HINTS}
    ${mvIMPACT_CHECK_INCLUDE_DIRS}
    NO_DEFAULT_PATH)
if(NOT mvIMPACT_INCLUDE_DIR OR NOT EXISTS ${mvIMPACT_INCLUDE_DIR})
    mvIMPACT_REPORT_NOT_FOUND(
        "Could not find mvimpact include directory, set mvIMPACT_INCLUDE_DIR to "
        "path to mvimpact include directory,"
        "e.g. /opt/mvIMPACT_acquire.")
else()
    message(STATUS "mvimpact include dir found: " ${mvIMPACT_INCLUDE_DIR})
endif()

# Find library directory for mvimpact
find_library(mvIMPACT_LIBRARY
    NAMES libmvBlueFOX.so
    PATHS ${mvIMPACT_LIBRARY_DIR_HINTS}
    ${mvIMPACT_CHECK_LIBRARY_DIRS}
    NO_DEFAULT_PATH)
if(NOT mvIMPACT_LIBRARY OR NOT EXISTS ${mvIMPACT_LIBRARY})
    mvIMPACT_REPORT_NOT_FOUND(
        "Could not find mvimpact library, set mvIMPACT_LIBRARY "
        "to full path to mvimpact library direcotory.${mvIMPACT_CHECK_LIBRARY_DIRS}~~")
else()
    # TODO: need to fix this hacky solution for getting mvIMPACT_LIBRARY_DIR
    string(REGEX MATCH ".*/" mvIMPACT_LIBRARY_DIR ${mvIMPACT_LIBRARY})
    message(STATUS "mvimpact library dir found: " ${mvIMPACT_LIBRARY_DIR})

endif()

# Mark internally as found, then verify. mvIMPACT_REPORT_NOT_FOUND() unsets if
# called.
set(mvIMPACT_FOUND TRUE)

# Extract mvimpact version
if(mvIMPACT_LIBRARY_DIR)
    file(GLOB mvIMPACT_LIBS
        RELATIVE ${mvIMPACT_LIBRARY_DIR}
        ${mvIMPACT_LIBRARY_DIR}/libmvBlueFOX.so.[0-9].[0-9].[0-9])
    # TODO: add version support
    # string(REGEX MATCH ""
    #       mvIMPACT_WORLD_VERSION ${mvIMPACT_PVBASE})
    # message(STATUS "mvimpact world version: " ${mvIMPACT_WORLD_VERSION})
endif()

# Catch case when caller has set mvIMPACT_INCLUDE_DIR in the cache / GUI and
# thus FIND_[PATH/LIBRARY] are not called, but specified locations are
# invalid, otherwise we would report the library as found.
if(NOT mvIMPACT_INCLUDE_DIR AND NOT EXISTS ${mvIMPACT_INCLUDE_DIR}/mvIMPACT_CPP/mvIMPACT_acquire.h)
    mvIMPACT_REPORT_NOT_FOUND("Caller defined mvIMPACT_INCLUDE_DIR: "
        ${mvIMPACT_INCLUDE_DIR}
        " does not contain mvIMPACT_CPP/mvIMPACT_acquire.h header.")
endif()

# Set standard CMake FindPackage variables if found.
if(mvIMPACT_FOUND)
    set(mvIMPACT_INCLUDE_DIRS ${mvIMPACT_INCLUDE_DIR})
    file(GLOB mvIMPACT_LIBRARIES ${mvIMPACT_LIBRARY_DIR}lib*.so)
endif()

# Handle REQUIRED / QUIET optional arguments.
include(FindPackageHandleStandardArgs)
if(mvIMPACT_FOUND)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(Mvimpact DEFAULT_MSG
        mvIMPACT_INCLUDE_DIRS mvIMPACT_LIBRARIES)
endif()

# Only mark internal variables as advanced if we found mvimpact, otherwise
# leave it visible in the standard GUI for the user to set manually.
if(mvIMPACT_FOUND)
    mark_as_advanced(FORCE mvIMPACT_INCLUDE_DIR mvIMPACT_LIBRARY)
endif()
