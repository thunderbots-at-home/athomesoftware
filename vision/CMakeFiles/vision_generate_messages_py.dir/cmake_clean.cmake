FILE(REMOVE_RECURSE
  "CMakeFiles/vision_generate_messages_py"
  "devel/lib/python2.7/dist-packages/vision/msg/_RealObject.py"
  "devel/lib/python2.7/dist-packages/vision/srv/_Match.py"
  "devel/lib/python2.7/dist-packages/vision/srv/_GetObjectsInScene.py"
  "devel/lib/python2.7/dist-packages/vision/srv/_FindObject.py"
  "devel/lib/python2.7/dist-packages/vision/srv/_Contains.py"
  "devel/lib/python2.7/dist-packages/vision/msg/__init__.py"
  "devel/lib/python2.7/dist-packages/vision/srv/__init__.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/vision_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
