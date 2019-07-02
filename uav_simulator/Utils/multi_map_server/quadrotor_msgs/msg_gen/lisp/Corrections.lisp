; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude Corrections.msg.html

(cl:defclass <Corrections> (roslisp-msg-protocol:ros-message)
  ((kf_correction
    :reader kf_correction
    :initarg :kf_correction
    :type cl:float
    :initform 0.0)
   (angle_corrections
    :reader angle_corrections
    :initarg :angle_corrections
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Corrections (<Corrections>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Corrections>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Corrections)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<Corrections> is deprecated: use quadrotor_msgs-msg:Corrections instead.")))

(cl:ensure-generic-function 'kf_correction-val :lambda-list '(m))
(cl:defmethod kf_correction-val ((m <Corrections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:kf_correction-val is deprecated.  Use quadrotor_msgs-msg:kf_correction instead.")
  (kf_correction m))

(cl:ensure-generic-function 'angle_corrections-val :lambda-list '(m))
(cl:defmethod angle_corrections-val ((m <Corrections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:angle_corrections-val is deprecated.  Use quadrotor_msgs-msg:angle_corrections instead.")
  (angle_corrections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Corrections>) ostream)
  "Serializes a message object of type '<Corrections>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Corrections>) istream)
  "Deserializes a message object of type '<Corrections>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Corrections>)))
  "Returns string type for a message object of type '<Corrections>"
  "quadrotor_msgs/Corrections")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Corrections)))
  "Returns string type for a message object of type 'Corrections"
  "quadrotor_msgs/Corrections")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Corrections>)))
  "Returns md5sum for a message object of type '<Corrections>"
  "61e86887a75fe520847d3256306360f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Corrections)))
  "Returns md5sum for a message object of type 'Corrections"
  "61e86887a75fe520847d3256306360f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Corrections>)))
  "Returns full string definition for message of type '<Corrections>"
  (cl:format cl:nil "float64 kf_correction~%float64[2] angle_corrections~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Corrections)))
  "Returns full string definition for message of type 'Corrections"
  (cl:format cl:nil "float64 kf_correction~%float64[2] angle_corrections~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Corrections>))
  (cl:+ 0
     8
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'angle_corrections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Corrections>))
  "Converts a ROS message object to a list"
  (cl:list 'Corrections
    (cl:cons ':kf_correction (kf_correction msg))
    (cl:cons ':angle_corrections (angle_corrections msg))
))
