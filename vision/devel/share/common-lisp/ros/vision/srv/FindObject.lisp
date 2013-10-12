; Auto-generated. Do not edit!


(cl:in-package vision-srv)


;//! \htmlinclude FindObject-request.msg.html

(cl:defclass <FindObject-request> (roslisp-msg-protocol:ros-message)
  ((object
    :reader object
    :initarg :object
    :type cl:string
    :initform ""))
)

(cl:defclass FindObject-request (<FindObject-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FindObject-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FindObject-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-srv:<FindObject-request> is deprecated: use vision-srv:FindObject-request instead.")))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <FindObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-srv:object-val is deprecated.  Use vision-srv:object instead.")
  (object m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FindObject-request>) ostream)
  "Serializes a message object of type '<FindObject-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'object))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'object))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FindObject-request>) istream)
  "Deserializes a message object of type '<FindObject-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'object) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FindObject-request>)))
  "Returns string type for a service object of type '<FindObject-request>"
  "vision/FindObjectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindObject-request)))
  "Returns string type for a service object of type 'FindObject-request"
  "vision/FindObjectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FindObject-request>)))
  "Returns md5sum for a message object of type '<FindObject-request>"
  "b20c6a08bb4b6fbb03f43969b46e84e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FindObject-request)))
  "Returns md5sum for a message object of type 'FindObject-request"
  "b20c6a08bb4b6fbb03f43969b46e84e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FindObject-request>)))
  "Returns full string definition for message of type '<FindObject-request>"
  (cl:format cl:nil "string object~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FindObject-request)))
  "Returns full string definition for message of type 'FindObject-request"
  (cl:format cl:nil "string object~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FindObject-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'object))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FindObject-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FindObject-request
    (cl:cons ':object (object msg))
))
;//! \htmlinclude FindObject-response.msg.html

(cl:defclass <FindObject-response> (roslisp-msg-protocol:ros-message)
  ((real_object
    :reader real_object
    :initarg :real_object
    :type vision-msg:RealObject
    :initform (cl:make-instance 'vision-msg:RealObject)))
)

(cl:defclass FindObject-response (<FindObject-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FindObject-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FindObject-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-srv:<FindObject-response> is deprecated: use vision-srv:FindObject-response instead.")))

(cl:ensure-generic-function 'real_object-val :lambda-list '(m))
(cl:defmethod real_object-val ((m <FindObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-srv:real_object-val is deprecated.  Use vision-srv:real_object instead.")
  (real_object m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FindObject-response>) ostream)
  "Serializes a message object of type '<FindObject-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'real_object) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FindObject-response>) istream)
  "Deserializes a message object of type '<FindObject-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'real_object) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FindObject-response>)))
  "Returns string type for a service object of type '<FindObject-response>"
  "vision/FindObjectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindObject-response)))
  "Returns string type for a service object of type 'FindObject-response"
  "vision/FindObjectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FindObject-response>)))
  "Returns md5sum for a message object of type '<FindObject-response>"
  "b20c6a08bb4b6fbb03f43969b46e84e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FindObject-response)))
  "Returns md5sum for a message object of type 'FindObject-response"
  "b20c6a08bb4b6fbb03f43969b46e84e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FindObject-response>)))
  "Returns full string definition for message of type '<FindObject-response>"
  (cl:format cl:nil "vision/RealObject real_object~%~%~%================================================================================~%MSG: vision/RealObject~%Header header~%geometry_msgs/Point point~%sensor_msgs/Image picture~%string name~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FindObject-response)))
  "Returns full string definition for message of type 'FindObject-response"
  (cl:format cl:nil "vision/RealObject real_object~%~%~%================================================================================~%MSG: vision/RealObject~%Header header~%geometry_msgs/Point point~%sensor_msgs/Image picture~%string name~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FindObject-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'real_object))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FindObject-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FindObject-response
    (cl:cons ':real_object (real_object msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FindObject)))
  'FindObject-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FindObject)))
  'FindObject-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindObject)))
  "Returns string type for a service object of type '<FindObject>"
  "vision/FindObject")