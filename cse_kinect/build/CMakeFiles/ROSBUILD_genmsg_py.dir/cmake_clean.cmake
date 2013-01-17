FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/cse_kinect/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/cse_kinect/msg/__init__.py"
  "../src/cse_kinect/msg/_PoseData.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
