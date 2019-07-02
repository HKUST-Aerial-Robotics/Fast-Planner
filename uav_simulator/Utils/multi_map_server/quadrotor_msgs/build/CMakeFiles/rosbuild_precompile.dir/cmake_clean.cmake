FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/quadrotor_msgs/msg"
  "CMakeFiles/rosbuild_precompile"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rosbuild_precompile.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
