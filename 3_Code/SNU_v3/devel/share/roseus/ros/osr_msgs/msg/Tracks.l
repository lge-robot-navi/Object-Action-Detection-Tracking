;; Auto-generated. Do not edit!


(when (boundp 'osr_msgs::Tracks)
  (if (not (find-package "OSR_MSGS"))
    (make-package "OSR_MSGS"))
  (shadow 'Tracks (find-package "OSR_MSGS")))
(unless (find-package "OSR_MSGS::TRACKS")
  (make-package "OSR_MSGS::TRACKS"))

(in-package "ROS")
;;//! \htmlinclude Tracks.msg.html
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass osr_msgs::Tracks
  :super ros::object
  :slots (_header _odom _tracks ))

(defmethod osr_msgs::Tracks
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:odom __odom) (instance nav_msgs::Odometry :init))
    ((:tracks __tracks) (let (r) (dotimes (i 0) (push (instance osr_msgs::Track :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _odom __odom)
   (setq _tracks __tracks)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:odom
   (&rest __odom)
   (if (keywordp (car __odom))
       (send* _odom __odom)
     (progn
       (if __odom (setq _odom (car __odom)))
       _odom)))
  (:tracks
   (&rest __tracks)
   (if (keywordp (car __tracks))
       (send* _tracks __tracks)
     (progn
       (if __tracks (setq _tracks (car __tracks)))
       _tracks)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; nav_msgs/Odometry _odom
    (send _odom :serialization-length)
    ;; osr_msgs/Track[] _tracks
    (apply #'+ (send-all _tracks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; nav_msgs/Odometry _odom
       (send _odom :serialize s)
     ;; osr_msgs/Track[] _tracks
     (write-long (length _tracks) s)
     (dolist (elem _tracks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; nav_msgs/Odometry _odom
     (send _odom :deserialize buf ptr-) (incf ptr- (send _odom :serialization-length))
   ;; osr_msgs/Track[] _tracks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tracks (let (r) (dotimes (i n) (push (instance osr_msgs::Track :init) r)) r))
     (dolist (elem- _tracks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get osr_msgs::Tracks :md5sum-) "e904ed11bcc18b45d7cfb15a9dbaeb75")
(setf (get osr_msgs::Tracks :datatype-) "osr_msgs/Tracks")
(setf (get osr_msgs::Tracks :definition-)
      "####################
# osr_msgs tracks message
####################
# Header
Header header

# Robot Pose
nav_msgs/Odometry odom

# Track 
Track[] tracks


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: osr_msgs/Track
####################
# osr_msgs track message
####################
# ID
uint8 id

# Type (1: PERSON, 2: CAR)
uint8 type

# Posture (1: STAND, 2: SIT DOWN, 3: LIE DOWN)
uint8 posture

# Activity score
# float32 activity

# Bounding box
osr_msgs/BoundingBox bbox_pose
osr_msgs/BoundingBox bbox_velocity

# State
geometry_msgs/Pose pose
geometry_msgs/Twist twist

# 3D Bounding Box


================================================================================
MSG: osr_msgs/BoundingBox
####################
# osr_msgs bounding box message
####################

uint32 x
uint32 y 
uint32 height
uint32 width

")



(provide :osr_msgs/Tracks "e904ed11bcc18b45d7cfb15a9dbaeb75")


