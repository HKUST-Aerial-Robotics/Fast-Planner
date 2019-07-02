FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/quadrotor_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/quadrotor_msgs/msg/__init__.py"
  "../src/quadrotor_msgs/msg/_Serial.py"
  "../src/quadrotor_msgs/msg/_AuxCommand.py"
  "../src/quadrotor_msgs/msg/_PositionCommand.py"
  "../src/quadrotor_msgs/msg/_Corrections.py"
  "../src/quadrotor_msgs/msg/_StatusData.py"
  "../src/quadrotor_msgs/msg/_SO3Command.py"
  "../src/quadrotor_msgs/msg/_PPROutputData.py"
  "../src/quadrotor_msgs/msg/_TRPYCommand.py"
  "../src/quadrotor_msgs/msg/_OutputData.py"
  "../src/quadrotor_msgs/msg/_Gains.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
