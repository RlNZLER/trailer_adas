# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_trailer_localiszation_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED trailer_localiszation_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(trailer_localiszation_FOUND FALSE)
  elseif(NOT trailer_localiszation_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(trailer_localiszation_FOUND FALSE)
  endif()
  return()
endif()
set(_trailer_localiszation_CONFIG_INCLUDED TRUE)

# output package information
if(NOT trailer_localiszation_FIND_QUIETLY)
  message(STATUS "Found trailer_localiszation: 0.0.0 (${trailer_localiszation_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'trailer_localiszation' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${trailer_localiszation_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(trailer_localiszation_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${trailer_localiszation_DIR}/${_extra}")
endforeach()
