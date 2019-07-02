; Auto-generated. Do not edit!


(cl:in-package multi_map_server-msg)


;//! \htmlinclude SparseMap3D.msg.html

(cl:defclass <SparseMap3D> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (info
    :reader info
    :initarg :info
    :type nav_msgs-msg:MapMetaData
    :initform (cl:make-instance 'nav_msgs-msg:MapMetaData))
   (lists
    :reader lists
    :initarg :lists
    :type (cl:vector multi_map_server-msg:VerticalOccupancyGridList)
   :initform (cl:make-array 0 :element-type 'multi_map_server-msg:VerticalOccupancyGridList :initial-element (cl:make-instance 'multi_map_server-msg:VerticalOccupancyGridList))))
)

(cl:defclass SparseMap3D (<SparseMap3D>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SparseMap3D>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SparseMap3D)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multi_map_server-msg:<SparseMap3D> is deprecated: use multi_map_server-msg:SparseMap3D instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SparseMap3D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_map_server-msg:header-val is deprecated.  Use multi_map_server-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <SparseMap3D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_map_server-msg:info-val is deprecated.  Use multi_map_server-msg:info instead.")
  (info m))

(cl:ensure-generic-function 'lists-val :lambda-list '(m))
(cl:defmethod lists-val ((m <SparseMap3D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_map_server-msg:lists-val is deprecated.  Use multi_map_server-msg:lists instead.")
  (lists m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SparseMap3D>) ostream)
  "Serializes a message object of type '<SparseMap3D>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'info) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'lists))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'lists))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SparseMap3D>) istream)
  "Deserializes a message object of type '<SparseMap3D>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'info) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'lists) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'lists)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'multi_map_server-msg:VerticalOccupancyGridList))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SparseMap3D>)))
  "Returns string type for a message object of type '<SparseMap3D>"
  "multi_map_server/SparseMap3D")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SparseMap3D)))
  "Returns string type for a message object of type 'SparseMap3D"
  "multi_map_server/SparseMap3D")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SparseMap3D>)))
  "Returns md5sum for a message object of type '<SparseMap3D>"
  "a20102f0b3a02e95070dab4140b78fb5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SparseMap3D)))
  "Returns md5sum for a message object of type 'SparseMap3D"
  "a20102f0b3a02e95070dab4140b78fb5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SparseMap3D>)))
  "Returns full string definition for message of type '<SparseMap3D>"
  (cl:format cl:nil "Header header~%nav_msgs/MapMetaData info~%VerticalOccupancyGridList[] lists~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: multi_map_server/VerticalOccupancyGridList~%float32 x~%float32 y~%int32[] upper~%int32[] lower~%int32[] mass~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SparseMap3D)))
  "Returns full string definition for message of type 'SparseMap3D"
  (cl:format cl:nil "Header header~%nav_msgs/MapMetaData info~%VerticalOccupancyGridList[] lists~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: multi_map_server/VerticalOccupancyGridList~%float32 x~%float32 y~%int32[] upper~%int32[] lower~%int32[] mass~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SparseMap3D>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'info))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'lists) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SparseMap3D>))
  "Converts a ROS message object to a list"
  (cl:list 'SparseMap3D
    (cl:cons ':header (header msg))
    (cl:cons ':info (info msg))
    (cl:cons ':lists (lists msg))
))
