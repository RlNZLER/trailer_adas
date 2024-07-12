# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_single_trailer_assembly_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED single_trailer_assembly_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(single_trailer_assembly_FOUND FALSE)
  elseif(NOT single_trailer_assembly_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(single_trailer_assembly_FOUND FALSE)
  endif()
  return()
endif()
set(_single_trailer_assembly_CONFIG_INCLUDED TRUE)

# output package information
if(NOT single_trailer_assembly_FIND_QUIETLY)
  message(STATUS "Found single_trailer_assembly: 0.0.0 (${single_trailer_assembly_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'single_trailer_assembly' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${single_trailer_assembly_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(single_trailer_assembly_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${single_trailer_assembly_DIR}/${_extra}")
endforeach()
