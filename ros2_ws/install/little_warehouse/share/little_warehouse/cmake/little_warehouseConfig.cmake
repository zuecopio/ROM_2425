# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_little_warehouse_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED little_warehouse_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(little_warehouse_FOUND FALSE)
  elseif(NOT little_warehouse_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(little_warehouse_FOUND FALSE)
  endif()
  return()
endif()
set(_little_warehouse_CONFIG_INCLUDED TRUE)

# output package information
if(NOT little_warehouse_FIND_QUIETLY)
  message(STATUS "Found little_warehouse: 1.0.0 (${little_warehouse_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'little_warehouse' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${little_warehouse_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(little_warehouse_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${little_warehouse_DIR}/${_extra}")
endforeach()
