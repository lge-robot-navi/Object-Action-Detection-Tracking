; Auto-generated. Do not edit!


(cl:in-package osr_msgs-msg)


;//! \htmlinclude Tracks.msg.html

(cl:defclass <Tracks> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (odom
    :reader odom
    :initarg :odom
    :type nav_msgs-msg:Odometry
    :initform (cl:make-instance 'nav_msgs-msg:Odometry))
   (tracks
    :reader tracks
    :initarg :tracks
    :type (cl:vector osr_msgs-msg:Track)
   :initform (cl:make-array 0 :element-type 'osr_msgs-msg:Track :initial-element (cl:make-instance 'osr_msgs-msg:Track))))
)

(cl:defclass Tracks (<Tracks>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tracks>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tracks)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name osr_msgs-msg:<Tracks> is deprecated: use osr_msgs-msg:Tracks instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Tracks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader osr_msgs-msg:header-val is deprecated.  Use osr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'odom-val :lambda-list '(m))
(cl:defmethod odom-val ((m <Tracks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader osr_msgs-msg:odom-val is deprecated.  Use osr_msgs-msg:odom instead.")
  (odom m))

(cl:ensure-generic-function 'tracks-val :lambda-list '(m))
(cl:defmethod tracks-val ((m <Tracks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader osr_msgs-msg:tracks-val is deprecated.  Use osr_msgs-msg:tracks instead.")
  (tracks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tracks>) ostream)
  "Serializes a message object of type '<Tracks>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'odom) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tracks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tracks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tracks>) istream)
  "Deserializes a message object of type '<Tracks>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'odom) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tracks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tracks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'osr_msgs-msg:Track))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tracks>)))
  "Returns string type for a message object of type '<Tracks>"
  "osr_msgs/Tracks")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tracks)))
  "Returns string type for a message object of type 'Tracks"
  "osr_msgs/Tracks")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tracks>)))
  "Returns md5sum for a message object of type '<Tracks>"
  "e904ed11bcc18b45d7cfb15a9dbaeb75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tracks)))
  "Returns md5sum for a message object of type 'Tracks"
  "e904ed11bcc18b45d7cfb15a9dbaeb75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tracks>)))
  "Returns full string definition for message of type '<Tracks>"
  (cl:format cl:nil "####################~%# osr_msgs tracks message~%####################~%# Header~%Header header~%~%# Robot Pose~%nav_msgs/Odometry odom~%~%# Track ~%Track[] tracks~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: osr_msgs/Track~%####################~%# osr_msgs track message~%####################~%# ID~%uint8 id~%~%# Type (1: PERSON, 2: CAR)~%uint8 type~%~%# Posture (1: STAND, 2: SIT DOWN, 3: LIE DOWN)~%uint8 posture~%~%# Activity score~%# float32 activity~%~%# Bounding box~%osr_msgs/BoundingBox bbox_pose~%osr_msgs/BoundingBox bbox_velocity~%~%# State~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%~%# 3D Bounding Box~%~%~%================================================================================~%MSG: osr_msgs/BoundingBox~%####################~%# osr_msgs bounding box message~%####################~%~%uint32 x~%uint32 y ~%uint32 height~%uint32 width~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tracks)))
  "Returns full string definition for message of type 'Tracks"
  (cl:format cl:nil "####################~%# osr_msgs tracks message~%####################~%# Header~%Header header~%~%# Robot Pose~%nav_msgs/Odometry odom~%~%# Track ~%Track[] tracks~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: osr_msgs/Track~%####################~%# osr_msgs track message~%####################~%# ID~%uint8 id~%~%# Type (1: PERSON, 2: CAR)~%uint8 type~%~%# Posture (1: STAND, 2: SIT DOWN, 3: LIE DOWN)~%uint8 posture~%~%# Activity score~%# float32 activity~%~%# Bounding box~%osr_msgs/BoundingBox bbox_pose~%osr_msgs/BoundingBox bbox_velocity~%~%# State~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%~%# 3D Bounding Box~%~%~%================================================================================~%MSG: osr_msgs/BoundingBox~%####################~%# osr_msgs bounding box message~%####################~%~%uint32 x~%uint32 y ~%uint32 height~%uint32 width~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tracks>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'odom))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tracks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tracks>))
  "Converts a ROS message object to a list"
  (cl:list 'Tracks
    (cl:cons ':header (header msg))
    (cl:cons ':odom (odom msg))
    (cl:cons ':tracks (tracks msg))
))
