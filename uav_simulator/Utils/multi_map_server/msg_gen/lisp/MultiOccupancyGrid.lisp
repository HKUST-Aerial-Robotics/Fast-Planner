; Auto-generated. Do not edit!


(cl:in-package multi_map_server-msg)


;//! \htmlinclude MultiOccupancyGrid.msg.html

(cl:defclass <MultiOccupancyGrid> (roslisp-msg-protocol:ros-message)
  ((maps
    :reader maps
    :initarg :maps
    :type (cl:vector nav_msgs-msg:OccupancyGrid)
   :initform (cl:make-array 0 :element-type 'nav_msgs-msg:OccupancyGrid :initial-element (cl:make-instance 'nav_msgs-msg:OccupancyGrid)))
   (origins
    :reader origins
    :initarg :origins
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass MultiOccupancyGrid (<MultiOccupancyGrid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MultiOccupancyGrid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MultiOccupancyGrid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multi_map_server-msg:<MultiOccupancyGrid> is deprecated: use multi_map_server-msg:MultiOccupancyGrid instead.")))

(cl:ensure-generic-function 'maps-val :lambda-list '(m))
(cl:defmethod maps-val ((m <MultiOccupancyGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_map_server-msg:maps-val is deprecated.  Use multi_map_server-msg:maps instead.")
  (maps m))

(cl:ensure-generic-function 'origins-val :lambda-list '(m))
(cl:defmethod origins-val ((m <MultiOccupancyGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_map_server-msg:origins-val is deprecated.  Use multi_map_server-msg:origins instead.")
  (origins m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MultiOccupancyGrid>) ostream)
  "Serializes a message object of type '<MultiOccupancyGrid>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'maps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'maps))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'origins))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'origins))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MultiOccupancyGrid>) istream)
  "Deserializes a message object of type '<MultiOccupancyGrid>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'maps) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'maps)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'nav_msgs-msg:OccupancyGrid))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'origins) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'origins)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MultiOccupancyGrid>)))
  "Returns string type for a message object of type '<MultiOccupancyGrid>"
  "multi_map_server/MultiOccupancyGrid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MultiOccupancyGrid)))
  "Returns string type for a message object of type 'MultiOccupancyGrid"
  "multi_map_server/MultiOccupancyGrid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MultiOccupancyGrid>)))
  "Returns md5sum for a message object of type '<MultiOccupancyGrid>"
  "61e63a291f11a6b1796a1edf79f34f72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MultiOccupancyGrid)))
  "Returns md5sum for a message object of type 'MultiOccupancyGrid"
  "61e63a291f11a6b1796a1edf79f34f72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MultiOccupancyGrid>)))
  "Returns full string definition for message of type '<MultiOccupancyGrid>"
  (cl:format cl:nil "nav_msgs/OccupancyGrid[] maps~%geometry_msgs/Pose[] origins~%~%================================================================================~%MSG: nav_msgs/OccupancyGrid~%# This represents a 2-D grid map, in which each cell represents the probability of~%# occupancy.~%~%Header header ~%~%#MetaData for the map~%MapMetaData info~%~%# The map data, in row-major order, starting with (0,0).  Occupancy~%# probabilities are in the range [0,100].  Unknown is -1.~%int8[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MultiOccupancyGrid)))
  "Returns full string definition for message of type 'MultiOccupancyGrid"
  (cl:format cl:nil "nav_msgs/OccupancyGrid[] maps~%geometry_msgs/Pose[] origins~%~%================================================================================~%MSG: nav_msgs/OccupancyGrid~%# This represents a 2-D grid map, in which each cell represents the probability of~%# occupancy.~%~%Header header ~%~%#MetaData for the map~%MapMetaData info~%~%# The map data, in row-major order, starting with (0,0).  Occupancy~%# probabilities are in the range [0,100].  Unknown is -1.~%int8[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MultiOccupancyGrid>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'maps) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'origins) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MultiOccupancyGrid>))
  "Converts a ROS message object to a list"
  (cl:list 'MultiOccupancyGrid
    (cl:cons ':maps (maps msg))
    (cl:cons ':origins (origins msg))
))
