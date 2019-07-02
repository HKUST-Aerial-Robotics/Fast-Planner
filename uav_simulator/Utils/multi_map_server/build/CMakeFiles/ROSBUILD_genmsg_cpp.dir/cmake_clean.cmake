FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/multi_map_server/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/multi_map_server/MultiOccupancyGrid.h"
  "../msg_gen/cpp/include/multi_map_server/SparseMap3D.h"
  "../msg_gen/cpp/include/multi_map_server/MultiSparseMap3D.h"
  "../msg_gen/cpp/include/multi_map_server/VerticalOccupancyGridList.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
