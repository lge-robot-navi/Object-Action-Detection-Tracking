;; Auto-generated. Do not edit!


(when (boundp 'osr_msgs::Track)
  (if (not (find-package "OSR_MSGS"))
    (make-package "OSR_MSGS"))
  (shadow 'Track (find-package "OSR_MSGS")))
(unless (find-package "OSR_MSGS::TRACK")
  (make-package "OSR_MSGS::TRACK"))

(in-package "ROS")
;;//! \htmlinclude Track.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass osr_msgs::Track
  :super ros::object
  :slots (_id _type _posture _bbox_pose _bbox_velocity _pose _twist ))

(defmethod osr_msgs::Track
  (:init
   (&key
    ((:id __id) 0)
    ((:type __type) 0)
    ((:posture __posture) 0)
    ((:bbox_pose __bbox_pose) (instance osr_msgs::BoundingBox :init))
    ((:bbox_velocity __bbox_velocity) (instance osr_msgs::BoundingBox :init))
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:twist __twist) (instance geometry_msgs::Twist :init))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _type (round __type))
   (setq _posture (round __posture))
   (setq _bbox_pose __bbox_pose)
   (setq _bbox_velocity __bbox_velocity)
   (setq _pose __pose)
   (setq _twist __twist)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:posture
   (&optional __posture)
   (if __posture (setq _posture __posture)) _posture)
  (:bbox_pose
   (&rest __bbox_pose)
   (if (keywordp (car __bbox_pose))
       (send* _bbox_pose __bbox_pose)
     (progn
       (if __bbox_pose (setq _bbox_pose (car __bbox_pose)))
       _bbox_pose)))
  (:bbox_velocity
   (&rest __bbox_velocity)
   (if (keywordp (car __bbox_velocity))
       (send* _bbox_velocity __bbox_velocity)
     (progn
       (if __bbox_velocity (setq _bbox_velocity (car __bbox_velocity)))
       _bbox_velocity)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; uint8 _type
    1
    ;; uint8 _posture
    1
    ;; osr_msgs/BoundingBox _bbox_pose
    (send _bbox_pose :serialization-length)
    ;; osr_msgs/BoundingBox _bbox_velocity
    (send _bbox_velocity :serialization-length)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Twist _twist
    (send _twist :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; uint8 _type
       (write-byte _type s)
     ;; uint8 _posture
       (write-byte _posture s)
     ;; osr_msgs/BoundingBox _bbox_pose
       (send _bbox_pose :serialize s)
     ;; osr_msgs/BoundingBox _bbox_velocity
       (send _bbox_velocity :serialize s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Twist _twist
       (send _twist :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _posture
     (setq _posture (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; osr_msgs/BoundingBox _bbox_pose
     (send _bbox_pose :deserialize buf ptr-) (incf ptr- (send _bbox_pose :serialization-length))
   ;; osr_msgs/BoundingBox _bbox_velocity
     (send _bbox_velocity :deserialize buf ptr-) (incf ptr- (send _bbox_velocity :serialization-length))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Twist _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;;
   self)
  )

(setf (get osr_msgs::Track :md5sum-) "a5743d60b28dea66cea0aa710dada021")
(setf (get osr_msgs::Track :datatype-) "osr_msgs/Track")
(setf (get osr_msgs::Track :definition-)
      "####################
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
")



(provide :osr_msgs/Track "a5743d60b28dea66cea0aa710dada021")


