#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "khi2cpp_hw::main" for configuration ""
set_property(TARGET khi2cpp_hw::main APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(khi2cpp_hw::main PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/main"
  )

list(APPEND _IMPORT_CHECK_TARGETS khi2cpp_hw::main )
list(APPEND _IMPORT_CHECK_FILES_FOR_khi2cpp_hw::main "${_IMPORT_PREFIX}/bin/main" )

# Import target "khi2cpp_hw::khi2cpp_hw" for configuration ""
set_property(TARGET khi2cpp_hw::khi2cpp_hw APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(khi2cpp_hw::khi2cpp_hw PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libkhi2cpp_hw.so"
  IMPORTED_SONAME_NOCONFIG "libkhi2cpp_hw.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS khi2cpp_hw::khi2cpp_hw )
list(APPEND _IMPORT_CHECK_FILES_FOR_khi2cpp_hw::khi2cpp_hw "${_IMPORT_PREFIX}/lib/libkhi2cpp_hw.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
