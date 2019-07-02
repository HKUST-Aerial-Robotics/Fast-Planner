FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/quadrotor_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/quadrotor_msgs/Serial.h"
  "../msg_gen/cpp/include/quadrotor_msgs/AuxCommand.h"
  "../msg_gen/cpp/include/quadrotor_msgs/PositionCommand.h"
  "../msg_gen/cpp/include/quadrotor_msgs/Corrections.h"
  "../msg_gen/cpp/include/quadrotor_msgs/StatusData.h"
  "../msg_gen/cpp/include/quadrotor_msgs/SO3Command.h"
  "../msg_gen/cpp/include/quadrotor_msgs/PPROutputData.h"
  "../msg_gen/cpp/include/quadrotor_msgs/TRPYCommand.h"
  "../msg_gen/cpp/include/quadrotor_msgs/OutputData.h"
  "../msg_gen/cpp/include/quadrotor_msgs/Gains.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
