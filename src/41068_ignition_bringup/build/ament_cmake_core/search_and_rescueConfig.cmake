# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_search_and_rescue_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED search_and_rescue_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(search_and_rescue_FOUND FALSE)
  elseif(NOT search_and_rescue_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(search_and_rescue_FOUND FALSE)
  endif()
  return()
endif()
set(_search_and_rescue_CONFIG_INCLUDED TRUE)

# output package information
if(NOT search_and_rescue_FIND_QUIETLY)
  message(STATUS "Found search_and_rescue: 1.0.3 (${search_and_rescue_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'search_and_rescue' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${search_and_rescue_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(search_and_rescue_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${search_and_rescue_DIR}/${_extra}")
endforeach()
