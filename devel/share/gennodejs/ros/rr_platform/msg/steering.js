// Auto-generated. Do not edit!

// (in-package rr_platform.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class steering {
  constructor() {
    this.header = new std_msgs.msg.Header();
    this.angle = 0.0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type steering
    // Serialize message field [header]
    bufferInfo = std_msgs.msg.Header.serialize(obj.header, bufferInfo);
    // Serialize message field [angle]
    bufferInfo = _serializer.float64(obj.angle, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type steering
    let tmp;
    let len;
    let data = new steering();
    // Deserialize message field [header]
    tmp = std_msgs.msg.Header.deserialize(buffer);
    data.header = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [angle]
    tmp = _deserializer.float64(buffer);
    data.angle = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'rr_platform/steering';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '84c1d14f72a90efbf3b1a4de632c5bfb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message for controlling the RC car servo and motors
    Header header           # timestamp in the header is the time the command is sent
    float64 angle             # front wheel angle
    
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
    
    `;
  }

};

module.exports = steering;
