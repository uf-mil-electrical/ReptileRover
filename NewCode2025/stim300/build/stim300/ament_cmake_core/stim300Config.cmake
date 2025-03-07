# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_stim300_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED stim300_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(stim300_FOUND FALSE)
  elseif(NOT stim300_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(stim300_FOUND FALSE)
  endif()
  return()
endif()
set(_stim300_CONFIG_INCLUDED TRUE)

# output package information
if(NOT stim300_FIND_QUIETLY)
  message(STATUS "Found stim300: 0.0.0 (${stim300_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'stim300' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT stim300_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(stim300_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${stim300_DIR}/${_extra}")
endforeach()
