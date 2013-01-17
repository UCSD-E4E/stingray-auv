FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/os5000/os5000Config.h"
  "../docs/os5000Config.dox"
  "../docs/os5000Config-usage.dox"
  "../src/os5000/cfg/os5000Config.py"
  "../docs/os5000Config.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
