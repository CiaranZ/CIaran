file(REMOVE_RECURSE
  "doc/doxygen-html"
  "doc/doxygen.log"
  "doc/pinocchio.doxytag"
  "CMakeFiles/Nightly"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/Nightly.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()