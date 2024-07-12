# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_single-trailer-assembly_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED single-trailer-assembly_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(single-trailer-assembly_FOUND FALSE)
  elseif(NOT single-trailer-assembly_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(single-trailer-assembly_FOUND FALSE)
  endif()
  return()
endif()
set(_single-trailer-assembly_CONFIG_INCLUDED TRUE)

# output package information
if(NOT single-trailer-assembly_FIND_QUIETLY)
  message(STATUS "Found single-trailer-assembly: 0.0.0 (${single-trailer-assembly_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'single-trailer-assembly' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${single-trailer-assembly_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(single-trailer-assembly_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${single-trailer-assembly_DIR}/${_extra}")
endforeach()
