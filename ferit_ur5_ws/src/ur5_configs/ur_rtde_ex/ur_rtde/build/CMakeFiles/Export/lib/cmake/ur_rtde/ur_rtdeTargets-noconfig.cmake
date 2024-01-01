#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ur_rtde::rtde" for configuration ""
set_property(TARGET ur_rtde::rtde APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ur_rtde::rtde PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librtde.so.1.5.5"
  IMPORTED_SONAME_NOCONFIG "librtde.so.1.5"
  )

list(APPEND _IMPORT_CHECK_TARGETS ur_rtde::rtde )
list(APPEND _IMPORT_CHECK_FILES_FOR_ur_rtde::rtde "${_IMPORT_PREFIX}/lib/librtde.so.1.5.5" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
