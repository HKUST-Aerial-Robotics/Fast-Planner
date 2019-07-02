; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude Serial.msg.html

(cl:defclass <Serial> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (channel
    :reader channel
    :initarg :channel
    :type cl:fixnum
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Serial (<Serial>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Serial>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Serial)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<Serial> is deprecated: use quadrotor_msgs-msg:Serial instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Serial>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'channel-val :lambda-list '(m))
(cl:defmethod channel-val ((m <Serial>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:channel-val is deprecated.  Use quadrotor_msgs-msg:channel instead.")
  (channel m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Serial>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:type-val is deprecated.  Use quadrotor_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Serial>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:data-val is deprecated.  Use quadrotor_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Serial>)))
    "Constants for message type '<Serial>"
  '((:SO3_CMD . 115)
    (:TRPY_CMD . 112)
    (:STATUS_DATA . 99)
    (:OUTPUT_DATA . 100)
    (:PPR_OUTPUT_DATA . 116)
    (:PPR_GAINS . 103))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Serial)))
    "Constants for message type 'Serial"
  '((:SO3_CMD . 115)
    (:TRPY_CMD . 112)
    (:STATUS_DATA . 99)
    (:OUTPUT_DATA . 100)
    (:PPR_OUTPUT_DATA . 116)
    (:PPR_GAINS . 103))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Serial>) ostream)
  "Serializes a message object of type '<Serial>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'channel)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Serial>) istream)
  "Deserializes a message object of type '<Serial>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'channel)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Serial>)))
  "Returns string type for a message object of type '<Serial>"
  "quadrotor_msgs/Serial")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Serial)))
  "Returns string type for a message object of type 'Serial"
  "quadrotor_msgs/Serial")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Serial>)))
  "Returns md5sum for a message object of type '<Serial>"
  "e448fb7595af9a8adfcab5ec241c7d4f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Serial)))
  "Returns md5sum for a message object of type 'Serial"
  "e448fb7595af9a8adfcab5ec241c7d4f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Serial>)))
  "Returns full string definition for message of type '<Serial>"
  (cl:format cl:nil "# Note: These constants need to be kept in sync with the types~%# defined in include/quadrotor_msgs/comm_types.h~%uint8 SO3_CMD = 115 # 's' in base 10~%uint8 TRPY_CMD = 112 # 'p' in base 10~%uint8 STATUS_DATA = 99 # 'c' in base 10~%uint8 OUTPUT_DATA = 100 # 'd' in base 10~%uint8 PPR_OUTPUT_DATA = 116 # 't' in base 10~%uint8 PPR_GAINS = 103 # 'g'~%~%Header header~%uint8 channel~%uint8 type # One of the types listed above~%uint8[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Serial)))
  "Returns full string definition for message of type 'Serial"
  (cl:format cl:nil "# Note: These constants need to be kept in sync with the types~%# defined in include/quadrotor_msgs/comm_types.h~%uint8 SO3_CMD = 115 # 's' in base 10~%uint8 TRPY_CMD = 112 # 'p' in base 10~%uint8 STATUS_DATA = 99 # 'c' in base 10~%uint8 OUTPUT_DATA = 100 # 'd' in base 10~%uint8 PPR_OUTPUT_DATA = 116 # 't' in base 10~%uint8 PPR_GAINS = 103 # 'g'~%~%Header header~%uint8 channel~%uint8 type # One of the types listed above~%uint8[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Serial>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Serial>))
  "Converts a ROS message object to a list"
  (cl:list 'Serial
    (cl:cons ':header (header msg))
    (cl:cons ':channel (channel msg))
    (cl:cons ':type (type msg))
    (cl:cons ':data (data msg))
))
