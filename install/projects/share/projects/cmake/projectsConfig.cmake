# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_projects_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED projects_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(projects_FOUND FALSE)
  elseif(NOT projects_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(projects_FOUND FALSE)
  endif()
  return()
endif()
set(_projects_CONFIG_INCLUDED TRUE)

# output package information
if(NOT projects_FIND_QUIETLY)
  message(STATUS "Found projects: 1.0.0 (${projects_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'projects' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${projects_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(projects_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${projects_DIR}/${_extra}")
endforeach()
