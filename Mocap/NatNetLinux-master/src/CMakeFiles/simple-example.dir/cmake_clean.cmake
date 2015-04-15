FILE(REMOVE_RECURSE
  "CMakeFiles/simple-example.dir/SimpleExample.cpp.o"
  "../bin/simple-example.pdb"
  "../bin/simple-example"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/simple-example.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
