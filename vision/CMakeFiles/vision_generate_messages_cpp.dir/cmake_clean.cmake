FILE(REMOVE_RECURSE
  "CMakeFiles/vision_generate_messages_cpp"
  "devel/include/vision/RealObject.h"
  "devel/include/vision/Match.h"
  "devel/include/vision/GetObjectsInScene.h"
  "devel/include/vision/FindObject.h"
  "devel/include/vision/Contains.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/vision_generate_messages_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
