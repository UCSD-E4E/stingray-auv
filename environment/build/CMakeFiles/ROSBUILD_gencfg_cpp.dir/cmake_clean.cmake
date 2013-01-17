FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/environment/environmentParamsConfig.h"
  "../docs/environmentParamsConfig.dox"
  "../docs/environmentParamsConfig-usage.dox"
  "../src/environment/cfg/environmentParamsConfig.py"
  "../docs/environmentParamsConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
