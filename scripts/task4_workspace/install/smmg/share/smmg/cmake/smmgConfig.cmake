# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_smmg_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED smmg_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(smmg_FOUND FALSE)
  elseif(NOT smmg_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(smmg_FOUND FALSE)
  endif()
  return()
endif()
set(_smmg_CONFIG_INCLUDED TRUE)

# output package information
if(NOT smmg_FIND_QUIETLY)
  message(STATUS "Found smmg: 0.3.0 (${smmg_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'smmg' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${smmg_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(smmg_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${smmg_DIR}/${_extra}")
endforeach()
