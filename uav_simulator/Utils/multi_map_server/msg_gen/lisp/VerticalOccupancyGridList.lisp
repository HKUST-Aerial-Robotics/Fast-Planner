; Auto-generated. Do not edit!


(cl:in-package multi_map_server-msg)


;//! \htmlinclude VerticalOccupancyGridList.msg.html

(cl:defclass <VerticalOccupancyGridList> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (upper
    :reader upper
    :initarg :upper
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (lower
    :reader lower
    :initarg :lower
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (mass
    :reader mass
    :initarg :mass
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass VerticalOccupancyGridList (<VerticalOccupancyGridList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VerticalOccupancyGridList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VerticalOccupancyGridList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multi_map_server-msg:<VerticalOccupancyGridList> is deprecated: use multi_map_server-msg:VerticalOccupancyGridList instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <VerticalOccupancyGridList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_map_server-msg:x-val is deprecated.  Use multi_map_server-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <VerticalOccupancyGridList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_map_server-msg:y-val is deprecated.  Use multi_map_server-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'upper-val :lambda-list '(m))
(cl:defmethod upper-val ((m <VerticalOccupancyGridList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_map_server-msg:upper-val is deprecated.  Use multi_map_server-msg:upper instead.")
  (upper m))

(cl:ensure-generic-function 'lower-val :lambda-list '(m))
(cl:defmethod lower-val ((m <VerticalOccupancyGridList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_map_server-msg:lower-val is deprecated.  Use multi_map_server-msg:lower instead.")
  (lower m))

(cl:ensure-generic-function 'mass-val :lambda-list '(m))
(cl:defmethod mass-val ((m <VerticalOccupancyGridList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_map_server-msg:mass-val is deprecated.  Use multi_map_server-msg:mass instead.")
  (mass m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VerticalOccupancyGridList>) ostream)
  "Serializes a message object of type '<VerticalOccupancyGridList>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'upper))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'upper))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'lower))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'lower))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mass))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'mass))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VerticalOccupancyGridList>) istream)
  "Deserializes a message object of type '<VerticalOccupancyGridList>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'upper) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'upper)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'lower) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'lower)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mass) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mass)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VerticalOccupancyGridList>)))
  "Returns string type for a message object of type '<VerticalOccupancyGridList>"
  "multi_map_server/VerticalOccupancyGridList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VerticalOccupancyGridList)))
  "Returns string type for a message object of type 'VerticalOccupancyGridList"
  "multi_map_server/VerticalOccupancyGridList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VerticalOccupancyGridList>)))
  "Returns md5sum for a message object of type '<VerticalOccupancyGridList>"
  "7ef85cc95b82747f51eb01a16bd7c795")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VerticalOccupancyGridList)))
  "Returns md5sum for a message object of type 'VerticalOccupancyGridList"
  "7ef85cc95b82747f51eb01a16bd7c795")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VerticalOccupancyGridList>)))
  "Returns full string definition for message of type '<VerticalOccupancyGridList>"
  (cl:format cl:nil "float32 x~%float32 y~%int32[] upper~%int32[] lower~%int32[] mass~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VerticalOccupancyGridList)))
  "Returns full string definition for message of type 'VerticalOccupancyGridList"
  (cl:format cl:nil "float32 x~%float32 y~%int32[] upper~%int32[] lower~%int32[] mass~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VerticalOccupancyGridList>))
  (cl:+ 0
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'upper) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'lower) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mass) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VerticalOccupancyGridList>))
  "Converts a ROS message object to a list"
  (cl:list 'VerticalOccupancyGridList
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':upper (upper msg))
    (cl:cons ':lower (lower msg))
    (cl:cons ':mass (mass msg))
))
