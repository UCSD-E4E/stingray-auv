FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/state_estimator/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/state_estimator/msg/__init__.py"
  "../src/state_estimator/msg/_PrimePowerStartStop.py"
  "../src/state_estimator/msg/_LoadTorque.py"
  "../src/state_estimator/msg/_SimDVLData.py"
  "../src/state_estimator/msg/_ControlInputs.py"
  "../src/state_estimator/msg/_StatePrediction.py"
  "../src/state_estimator/msg/_State.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
