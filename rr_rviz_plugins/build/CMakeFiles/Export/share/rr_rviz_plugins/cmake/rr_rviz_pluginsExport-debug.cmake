#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rr_rviz_plugins::rr_rviz_plugins" for configuration "Debug"
set_property(TARGET rr_rviz_plugins::rr_rviz_plugins APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(rr_rviz_plugins::rr_rviz_plugins PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/librr_rviz_plugins.so"
  IMPORTED_SONAME_DEBUG "librr_rviz_plugins.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rr_rviz_plugins::rr_rviz_plugins )
list(APPEND _IMPORT_CHECK_FILES_FOR_rr_rviz_plugins::rr_rviz_plugins "${_IMPORT_PREFIX}/lib/librr_rviz_plugins.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
