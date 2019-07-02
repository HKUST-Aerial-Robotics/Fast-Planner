; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude SO3Command.msg.html

(cl:defclass <SO3Command> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (force
    :reader force
    :initarg :force
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (orientation
    :reader orientation
    :initarg :orientation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (kR
    :reader kR
    :initarg :kR
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (kOm
    :reader kOm
    :initarg :kOm
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (aux
    :reader aux
    :initarg :aux
    :type quadrotor_msgs-msg:AuxCommand
    :initform (cl:make-instance 'quadrotor_msgs-msg:AuxCommand)))
)

(cl:defclass SO3Command (<SO3Command>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SO3Command>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SO3Command)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<SO3Command> is deprecated: use quadrotor_msgs-msg:SO3Command instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SO3Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <SO3Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:force-val is deprecated.  Use quadrotor_msgs-msg:force instead.")
  (force m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <SO3Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:orientation-val is deprecated.  Use quadrotor_msgs-msg:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'kR-val :lambda-list '(m))
(cl:defmethod kR-val ((m <SO3Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:kR-val is deprecated.  Use quadrotor_msgs-msg:kR instead.")
  (kR m))

(cl:ensure-generic-function 'kOm-val :lambda-list '(m))
(cl:defmethod kOm-val ((m <SO3Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:kOm-val is deprecated.  Use quadrotor_msgs-msg:kOm instead.")
  (kOm m))

(cl:ensure-generic-function 'aux-val :lambda-list '(m))
(cl:defmethod aux-val ((m <SO3Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:aux-val is deprecated.  Use quadrotor_msgs-msg:aux instead.")
  (aux m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SO3Command>) ostream)
  "Serializes a message object of type '<SO3Command>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'force) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'kR))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'kOm))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'aux) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SO3Command>) istream)
  "Deserializes a message object of type '<SO3Command>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'force) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
  (cl:setf (cl:slot-value msg 'kR) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'kR)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'kOm) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'kOm)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'aux) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SO3Command>)))
  "Returns string type for a message object of type '<SO3Command>"
  "quadrotor_msgs/SO3Command")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SO3Command)))
  "Returns string type for a message object of type 'SO3Command"
  "quadrotor_msgs/SO3Command")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SO3Command>)))
  "Returns md5sum for a message object of type '<SO3Command>"
  "a466650b2633e768513aa3bf62383c86")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SO3Command)))
  "Returns md5sum for a message object of type 'SO3Command"
  "a466650b2633e768513aa3bf62383c86")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SO3Command>)))
  "Returns full string definition for message of type '<SO3Command>"
  (cl:format cl:nil "Header header~%geometry_msgs/Vector3 force~%geometry_msgs/Quaternion orientation~%float64[3] kR~%float64[3] kOm~%quadrotor_msgs/AuxCommand aux~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: quadrotor_msgs/AuxCommand~%float64 current_yaw~%float64 kf_correction~%float64[2] angle_corrections# Trims for roll, pitch~%bool enable_motors~%bool use_external_yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SO3Command)))
  "Returns full string definition for message of type 'SO3Command"
  (cl:format cl:nil "Header header~%geometry_msgs/Vector3 force~%geometry_msgs/Quaternion orientation~%float64[3] kR~%float64[3] kOm~%quadrotor_msgs/AuxCommand aux~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: quadrotor_msgs/AuxCommand~%float64 current_yaw~%float64 kf_correction~%float64[2] angle_corrections# Trims for roll, pitch~%bool enable_motors~%bool use_external_yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SO3Command>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'force))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'kR) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'kOm) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'aux))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SO3Command>))
  "Converts a ROS message object to a list"
  (cl:list 'SO3Command
    (cl:cons ':header (header msg))
    (cl:cons ':force (force msg))
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':kR (kR msg))
    (cl:cons ':kOm (kOm msg))
    (cl:cons ':aux (aux msg))
))
