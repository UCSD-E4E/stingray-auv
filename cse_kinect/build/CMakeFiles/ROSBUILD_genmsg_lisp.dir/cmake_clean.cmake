FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/cse_kinect/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/PoseData.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_PoseData.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
