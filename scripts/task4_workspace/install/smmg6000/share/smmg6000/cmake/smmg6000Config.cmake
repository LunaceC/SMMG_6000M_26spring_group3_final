# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_smmg6000_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED smmg6000_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(smmg6000_FOUND FALSE)
  elseif(NOT smmg6000_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(smmg6000_FOUND FALSE)
  endif()
  return()
endif()
set(_smmg6000_CONFIG_INCLUDED TRUE)

# output package information
if(NOT smmg6000_FIND_QUIETLY)
  message(STATUS "Found smmg6000: 0.0.0 (${smmg6000_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'smmg6000' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${smmg6000_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(smmg6000_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${smmg6000_DIR}/${_extra}")
endforeach()
