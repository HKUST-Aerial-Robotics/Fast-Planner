FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/multi_map_server/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/multi_map_server/msg/__init__.py"
  "../src/multi_map_server/msg/_MultiOccupancyGrid.py"
  "../src/multi_map_server/msg/_SparseMap3D.py"
  "../src/multi_map_server/msg/_MultiSparseMap3D.py"
  "../src/multi_map_server/msg/_VerticalOccupancyGridList.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
