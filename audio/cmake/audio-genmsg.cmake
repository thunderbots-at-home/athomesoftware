# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "audio: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iaudio:/home/marco/catkin_ws/src/athomesoftware/audio/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(audio_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(audio
  "/home/marco/catkin_ws/src/athomesoftware/audio/msg/FRClientGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/audio
)

### Generating Services

### Generating Module File
_generate_module_cpp(audio
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/audio
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(audio_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(audio_generate_messages audio_generate_messages_cpp)

# target for backward compatibility
add_custom_target(audio_gencpp)
add_dependencies(audio_gencpp audio_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS audio_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(audio
  "/home/marco/catkin_ws/src/athomesoftware/audio/msg/FRClientGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/audio
)

### Generating Services

### Generating Module File
_generate_module_lisp(audio
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/audio
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(audio_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(audio_generate_messages audio_generate_messages_lisp)

# target for backward compatibility
add_custom_target(audio_genlisp)
add_dependencies(audio_genlisp audio_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS audio_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(audio
  "/home/marco/catkin_ws/src/athomesoftware/audio/msg/FRClientGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/audio
)

### Generating Services

### Generating Module File
_generate_module_py(audio
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/audio
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(audio_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(audio_generate_messages audio_generate_messages_py)

# target for backward compatibility
add_custom_target(audio_genpy)
add_dependencies(audio_genpy audio_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS audio_generate_messages_py)


debug_message(2 "audio: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/audio)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/audio
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/audio)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/audio
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/audio)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/audio\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/audio
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/audio
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "/audio/.+/__init__.pyc?$"
  )
endif()
