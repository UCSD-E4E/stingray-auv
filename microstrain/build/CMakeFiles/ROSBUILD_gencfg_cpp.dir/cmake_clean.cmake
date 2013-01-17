FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/microstrain/ms3dmgConfig.h"
  "../docs/ms3dmgConfig.dox"
  "../docs/ms3dmgConfig-usage.dox"
  "../src/microstrain/cfg/ms3dmgConfig.py"
  "../docs/ms3dmgConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
