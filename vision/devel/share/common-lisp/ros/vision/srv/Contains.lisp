; Auto-generated. Do not edit!


(cl:in-package vision-srv)


;//! \htmlinclude Contains-request.msg.html

(cl:defclass <Contains-request> (roslisp-msg-protocol:ros-message)
  ((object
    :reader object
    :initarg :object
    :type cl:string
    :initform ""))
)

(cl:defclass Contains-request (<Contains-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Contains-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Contains-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-srv:<Contains-request> is deprecated: use vision-srv:Contains-request instead.")))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <Contains-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-srv:object-val is deprecated.  Use vision-srv:object instead.")
  (object m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Contains-request>) ostream)
  "Serializes a message object of type '<Contains-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'object))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'object))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Contains-request>) istream)
  "Deserializes a message object of type '<Contains-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Contains-request>)))
  "Returns string type for a service object of type '<Contains-request>"
  "vision/ContainsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Contains-request)))
  "Returns string type for a service object of type 'Contains-request"
  "vision/ContainsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Contains-request>)))
  "Returns md5sum for a message object of type '<Contains-request>"
  "2c780fb23cef73bd4e865759d5092858")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Contains-request)))
  "Returns md5sum for a message object of type 'Contains-request"
  "2c780fb23cef73bd4e865759d5092858")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Contains-request>)))
  "Returns full string definition for message of type '<Contains-request>"
  (cl:format cl:nil "string object~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Contains-request)))
  "Returns full string definition for message of type 'Contains-request"
  (cl:format cl:nil "string object~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Contains-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'object))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Contains-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Contains-request
    (cl:cons ':object (object msg))
))
;//! \htmlinclude Contains-response.msg.html

(cl:defclass <Contains-response> (roslisp-msg-protocol:ros-message)
  ((contains
    :reader contains
    :initarg :contains
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Contains-response (<Contains-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Contains-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Contains-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-srv:<Contains-response> is deprecated: use vision-srv:Contains-response instead.")))

(cl:ensure-generic-function 'contains-val :lambda-list '(m))
(cl:defmethod contains-val ((m <Contains-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-srv:contains-val is deprecated.  Use vision-srv:contains instead.")
  (contains m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Contains-response>) ostream)
  "Serializes a message object of type '<Contains-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'contains) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Contains-response>) istream)
  "Deserializes a message object of type '<Contains-response>"
    (cl:setf (cl:slot-value msg 'contains) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Contains-response>)))
  "Returns string type for a service object of type '<Contains-response>"
  "vision/ContainsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Contains-response)))
  "Returns string type for a service object of type 'Contains-response"
  "vision/ContainsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Contains-response>)))
  "Returns md5sum for a message object of type '<Contains-response>"
  "2c780fb23cef73bd4e865759d5092858")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Contains-response)))
  "Returns md5sum for a message object of type 'Contains-response"
  "2c780fb23cef73bd4e865759d5092858")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Contains-response>)))
  "Returns full string definition for message of type '<Contains-response>"
  (cl:format cl:nil "bool contains~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Contains-response)))
  "Returns full string definition for message of type 'Contains-response"
  (cl:format cl:nil "bool contains~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Contains-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Contains-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Contains-response
    (cl:cons ':contains (contains msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Contains)))
  'Contains-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Contains)))
  'Contains-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Contains)))
  "Returns string type for a service object of type '<Contains>"
  "vision/Contains")