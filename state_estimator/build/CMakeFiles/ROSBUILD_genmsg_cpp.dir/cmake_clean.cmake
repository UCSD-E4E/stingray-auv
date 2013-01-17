FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/state_estimator/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/state_estimator/PrimePowerStartStop.h"
  "../msg_gen/cpp/include/state_estimator/LoadTorque.h"
  "../msg_gen/cpp/include/state_estimator/SimDVLData.h"
  "../msg_gen/cpp/include/state_estimator/ControlInputs.h"
  "../msg_gen/cpp/include/state_estimator/StatePrediction.h"
  "../msg_gen/cpp/include/state_estimator/State.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
