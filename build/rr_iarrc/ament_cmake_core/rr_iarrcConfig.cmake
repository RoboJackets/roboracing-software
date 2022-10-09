# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rr_iarrc_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rr_iarrc_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rr_iarrc_FOUND FALSE)
  elseif(NOT rr_iarrc_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rr_iarrc_FOUND FALSE)
  endif()
  return()
endif()
set(_rr_iarrc_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rr_iarrc_FIND_QUIETLY)
  message(STATUS "Found rr_iarrc: 0.0.0 (${rr_iarrc_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rr_iarrc' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rr_iarrc_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rr_iarrc_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rr_iarrc_DIR}/${_extra}")
endforeach()
