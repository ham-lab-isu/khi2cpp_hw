# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_khi2cpp_hw_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED khi2cpp_hw_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(khi2cpp_hw_FOUND FALSE)
  elseif(NOT khi2cpp_hw_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(khi2cpp_hw_FOUND FALSE)
  endif()
  return()
endif()
set(_khi2cpp_hw_CONFIG_INCLUDED TRUE)

# output package information
if(NOT khi2cpp_hw_FIND_QUIETLY)
  message(STATUS "Found khi2cpp_hw: 0.0.0 (${khi2cpp_hw_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'khi2cpp_hw' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${khi2cpp_hw_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(khi2cpp_hw_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${khi2cpp_hw_DIR}/${_extra}")
endforeach()
