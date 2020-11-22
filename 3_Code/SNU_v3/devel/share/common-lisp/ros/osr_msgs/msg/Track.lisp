; Auto-generated. Do not edit!


(cl:in-package osr_msgs-msg)


;//! \htmlinclude Track.msg.html

(cl:defclass <Track> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (posture
    :reader posture
    :initarg :posture
    :type cl:fixnum
    :initform 0)
   (bbox_pose
    :reader bbox_pose
    :initarg :bbox_pose
    :type osr_msgs-msg:BoundingBox
    :initform (cl:make-instance 'osr_msgs-msg:BoundingBox))
   (bbox_velocity
    :reader bbox_velocity
    :initarg :bbox_velocity
    :type osr_msgs-msg:BoundingBox
    :initform (cl:make-instance 'osr_msgs-msg:BoundingBox))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (twist
    :reader twist
    :initarg :twist
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist)))
)

(cl:defclass Track (<Track>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Track>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Track)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name osr_msgs-msg:<Track> is deprecated: use osr_msgs-msg:Track instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Track>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader osr_msgs-msg:id-val is deprecated.  Use osr_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Track>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader osr_msgs-msg:type-val is deprecated.  Use osr_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'posture-val :lambda-list '(m))
(cl:defmethod posture-val ((m <Track>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader osr_msgs-msg:posture-val is deprecated.  Use osr_msgs-msg:posture instead.")
  (posture m))

(cl:ensure-generic-function 'bbox_pose-val :lambda-list '(m))
(cl:defmethod bbox_pose-val ((m <Track>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader osr_msgs-msg:bbox_pose-val is deprecated.  Use osr_msgs-msg:bbox_pose instead.")
  (bbox_pose m))

(cl:ensure-generic-function 'bbox_velocity-val :lambda-list '(m))
(cl:defmethod bbox_velocity-val ((m <Track>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader osr_msgs-msg:bbox_velocity-val is deprecated.  Use osr_msgs-msg:bbox_velocity instead.")
  (bbox_velocity m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <Track>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader osr_msgs-msg:pose-val is deprecated.  Use osr_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'twist-val :lambda-list '(m))
(cl:defmethod twist-val ((m <Track>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader osr_msgs-msg:twist-val is deprecated.  Use osr_msgs-msg:twist instead.")
  (twist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Track>) ostream)
  "Serializes a message object of type '<Track>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'posture)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bbox_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bbox_velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'twist) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Track>) istream)
  "Deserializes a message object of type '<Track>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'posture)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bbox_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bbox_velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'twist) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Track>)))
  "Returns string type for a message object of type '<Track>"
  "osr_msgs/Track")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Track)))
  "Returns string type for a message object of type 'Track"
  "osr_msgs/Track")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Track>)))
  "Returns md5sum for a message object of type '<Track>"
  "a5743d60b28dea66cea0aa710dada021")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Track)))
  "Returns md5sum for a message object of type 'Track"
  "a5743d60b28dea66cea0aa710dada021")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Track>)))
  "Returns full string definition for message of type '<Track>"
  (cl:format cl:nil "####################~%# osr_msgs track message~%####################~%# ID~%uint8 id~%~%# Type (1: PERSON, 2: CAR)~%uint8 type~%~%# Posture (1: STAND, 2: SIT DOWN, 3: LIE DOWN)~%uint8 posture~%~%# Activity score~%# float32 activity~%~%# Bounding box~%osr_msgs/BoundingBox bbox_pose~%osr_msgs/BoundingBox bbox_velocity~%~%# State~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%~%# 3D Bounding Box~%~%~%================================================================================~%MSG: osr_msgs/BoundingBox~%####################~%# osr_msgs bounding box message~%####################~%~%uint32 x~%uint32 y ~%uint32 height~%uint32 width~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Track)))
  "Returns full string definition for message of type 'Track"
  (cl:format cl:nil "####################~%# osr_msgs track message~%####################~%# ID~%uint8 id~%~%# Type (1: PERSON, 2: CAR)~%uint8 type~%~%# Posture (1: STAND, 2: SIT DOWN, 3: LIE DOWN)~%uint8 posture~%~%# Activity score~%# float32 activity~%~%# Bounding box~%osr_msgs/BoundingBox bbox_pose~%osr_msgs/BoundingBox bbox_velocity~%~%# State~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%~%# 3D Bounding Box~%~%~%================================================================================~%MSG: osr_msgs/BoundingBox~%####################~%# osr_msgs bounding box message~%####################~%~%uint32 x~%uint32 y ~%uint32 height~%uint32 width~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Track>))
  (cl:+ 0
     1
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bbox_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bbox_velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'twist))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Track>))
  "Converts a ROS message object to a list"
  (cl:list 'Track
    (cl:cons ':id (id msg))
    (cl:cons ':type (type msg))
    (cl:cons ':posture (posture msg))
    (cl:cons ':bbox_pose (bbox_pose msg))
    (cl:cons ':bbox_velocity (bbox_velocity msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':twist (twist msg))
))
