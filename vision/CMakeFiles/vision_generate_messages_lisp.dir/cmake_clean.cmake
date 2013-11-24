FILE(REMOVE_RECURSE
  "CMakeFiles/vision_generate_messages_lisp"
  "devel/share/common-lisp/ros/vision/msg/RealObject.lisp"
  "devel/share/common-lisp/ros/vision/srv/Match.lisp"
  "devel/share/common-lisp/ros/vision/srv/GetObjectsInScene.lisp"
  "devel/share/common-lisp/ros/vision/srv/FindObject.lisp"
  "devel/share/common-lisp/ros/vision/srv/Contains.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/vision_generate_messages_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
