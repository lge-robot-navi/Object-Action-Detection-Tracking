// Auto-generated. Do not edit!

// (in-package osr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BoundingBox = require('./BoundingBox.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Track {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.type = null;
      this.posture = null;
      this.bbox_pose = null;
      this.bbox_velocity = null;
      this.pose = null;
      this.twist = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('posture')) {
        this.posture = initObj.posture
      }
      else {
        this.posture = 0;
      }
      if (initObj.hasOwnProperty('bbox_pose')) {
        this.bbox_pose = initObj.bbox_pose
      }
      else {
        this.bbox_pose = new BoundingBox();
      }
      if (initObj.hasOwnProperty('bbox_velocity')) {
        this.bbox_velocity = initObj.bbox_velocity
      }
      else {
        this.bbox_velocity = new BoundingBox();
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('twist')) {
        this.twist = initObj.twist
      }
      else {
        this.twist = new geometry_msgs.msg.Twist();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Track
    // Serialize message field [id]
    bufferOffset = _serializer.uint8(obj.id, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.uint8(obj.type, buffer, bufferOffset);
    // Serialize message field [posture]
    bufferOffset = _serializer.uint8(obj.posture, buffer, bufferOffset);
    // Serialize message field [bbox_pose]
    bufferOffset = BoundingBox.serialize(obj.bbox_pose, buffer, bufferOffset);
    // Serialize message field [bbox_velocity]
    bufferOffset = BoundingBox.serialize(obj.bbox_velocity, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [twist]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.twist, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Track
    let len;
    let data = new Track(null);
    // Deserialize message field [id]
    data.id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [posture]
    data.posture = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [bbox_pose]
    data.bbox_pose = BoundingBox.deserialize(buffer, bufferOffset);
    // Deserialize message field [bbox_velocity]
    data.bbox_velocity = BoundingBox.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [twist]
    data.twist = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 139;
  }

  static datatype() {
    // Returns string type for a message object
    return 'osr_msgs/Track';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a5743d60b28dea66cea0aa710dada021';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Track(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.posture !== undefined) {
      resolved.posture = msg.posture;
    }
    else {
      resolved.posture = 0
    }

    if (msg.bbox_pose !== undefined) {
      resolved.bbox_pose = BoundingBox.Resolve(msg.bbox_pose)
    }
    else {
      resolved.bbox_pose = new BoundingBox()
    }

    if (msg.bbox_velocity !== undefined) {
      resolved.bbox_velocity = BoundingBox.Resolve(msg.bbox_velocity)
    }
    else {
      resolved.bbox_velocity = new BoundingBox()
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.twist !== undefined) {
      resolved.twist = geometry_msgs.msg.Twist.Resolve(msg.twist)
    }
    else {
      resolved.twist = new geometry_msgs.msg.Twist()
    }

    return resolved;
    }
};

module.exports = Track;
