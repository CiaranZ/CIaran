#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pinocchio::pinocchio" for configuration ""
set_property(TARGET pinocchio::pinocchio APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(pinocchio::pinocchio PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so;/usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so;/usr/lib/x86_64-linux-gnu/liburdfdom_model.so;/usr/lib/x86_64-linux-gnu/liburdfdom_world.so;console_bridge::console_bridge;hpp-fcl::hpp-fcl"
  IMPORTED_LOCATION_NOCONFIG "/usr/local/lib/libpinocchio.so.2.5.1"
  IMPORTED_SONAME_NOCONFIG "libpinocchio.so.2.5.1"
  )

list(APPEND _cmake_import_check_targets pinocchio::pinocchio )
list(APPEND _cmake_import_check_files_for_pinocchio::pinocchio "/usr/local/lib/libpinocchio.so.2.5.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
