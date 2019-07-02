; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude AuxCommand.msg.html

(cl:defclass <AuxCommand> (roslisp-msg-protocol:ros-message)
  ((current_yaw
    :reader current_yaw
    :initarg :current_yaw
    :type cl:float
    :initform 0.0)
   (kf_correction
    :reader kf_correction
    :initarg :kf_correction
    :type cl:float
    :initform 0.0)
   (angle_corrections
    :reader angle_corrections
    :initarg :angle_corrections
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (enable_motors
    :reader enable_motors
    :initarg :enable_motors
    :type cl:boolean
    :initform cl:nil)
   (use_external_yaw
    :reader use_external_yaw
    :initarg :use_external_yaw
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AuxCommand (<AuxCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AuxCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AuxCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<AuxCommand> is deprecated: use quadrotor_msgs-msg:AuxCommand instead.")))

(cl:ensure-generic-function 'current_yaw-val :lambda-list '(m))
(cl:defmethod current_yaw-val ((m <AuxCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:current_yaw-val is deprecated.  Use quadrotor_msgs-msg:current_yaw instead.")
  (current_yaw m))

(cl:ensure-generic-function 'kf_correction-val :lambda-list '(m))
(cl:defmethod kf_correction-val ((m <AuxCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:kf_correction-val is deprecated.  Use quadrotor_msgs-msg:kf_correction instead.")
  (kf_correction m))

(cl:ensure-generic-function 'angle_corrections-val :lambda-list '(m))
(cl:defmethod angle_corrections-val ((m <AuxCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:angle_corrections-val is deprecated.  Use quadrotor_msgs-msg:angle_corrections instead.")
  (angle_corrections m))

(cl:ensure-generic-function 'enable_motors-val :lambda-list '(m))
(cl:defmethod enable_motors-val ((m <AuxCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:enable_motors-val is deprecated.  Use quadrotor_msgs-msg:enable_motors instead.")
  (enable_motors m))

(cl:ensure-generic-function 'use_external_yaw-val :lambda-list '(m))
(cl:defmethod use_external_yaw-val ((m <AuxCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:use_external_yaw-val is deprecated.  Use quadrotor_msgs-msg:use_external_yaw instead.")
  (use_external_yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AuxCommand>) ostream)
  "Serializes a message object of type '<AuxCommand>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'current_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'kf_correction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'angle_corrections))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable_motors) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_external_yaw) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AuxCommand>) istream)
  "Deserializes a message object of type '<AuxCommand>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_yaw) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kf_correction) (roslisp-utils:decode-double-float-bits bits)))
  (cl:setf (cl:slot-value msg 'angle_corrections) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'angle_corrections)))
    (cl:dotimes (i 2)
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
    (cl:setf (cl:slot-value msg 'enable_motors) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'use_external_yaw) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AuxCommand>)))
  "Returns string type for a message object of type '<AuxCommand>"
  "quadrotor_msgs/AuxCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AuxCommand)))
  "Returns string type for a message object of type 'AuxCommand"
  "quadrotor_msgs/AuxCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AuxCommand>)))
  "Returns md5sum for a message object of type '<AuxCommand>"
  "94f75840e4b1e03675da764692f2c839")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AuxCommand)))
  "Returns md5sum for a message object of type 'AuxCommand"
  "94f75840e4b1e03675da764692f2c839")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AuxCommand>)))
  "Returns full string definition for message of type '<AuxCommand>"
  (cl:format cl:nil "float64 current_yaw~%float64 kf_correction~%float64[2] angle_corrections# Trims for roll, pitch~%bool enable_motors~%bool use_external_yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AuxCommand)))
  "Returns full string definition for message of type 'AuxCommand"
  (cl:format cl:nil "float64 current_yaw~%float64 kf_correction~%float64[2] angle_corrections# Trims for roll, pitch~%bool enable_motors~%bool use_external_yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AuxCommand>))
  (cl:+ 0
     8
     8
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'angle_corrections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AuxCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'AuxCommand
    (cl:cons ':current_yaw (current_yaw msg))
    (cl:cons ':kf_correction (kf_correction msg))
    (cl:cons ':angle_corrections (angle_corrections msg))
    (cl:cons ':enable_motors (enable_motors msg))
    (cl:cons ':use_external_yaw (use_external_yaw msg))
))
