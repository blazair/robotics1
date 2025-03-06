# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_descy_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED descy_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(descy_FOUND FALSE)
  elseif(NOT descy_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(descy_FOUND FALSE)
  endif()
  return()
endif()
set(_descy_CONFIG_INCLUDED TRUE)

# output package information
if(NOT descy_FIND_QUIETLY)
  message(STATUS "Found descy: 0.0.0 (${descy_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'descy' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT descy_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(descy_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${descy_DIR}/${_extra}")
endforeach()
