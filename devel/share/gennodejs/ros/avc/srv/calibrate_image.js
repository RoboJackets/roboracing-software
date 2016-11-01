// Auto-generated. Do not edit!

// (in-package avc.srv)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class calibrate_imageRequest {
  constructor() {
    this.image = new sensor_msgs.msg.Image();
    this.imgDim = new Array(2).fill(0);
    this.pixelsPerMeter = 0.0;
    this.distanceToChessboard = 0.0;
    this.chessboardDim = new Array(2).fill(0);
    this.squareWidth = 0.0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type calibrate_imageRequest
    // Serialize message field [image]
    bufferInfo = sensor_msgs.msg.Image.serialize(obj.image, bufferInfo);
    // Serialize message field [imgDim]
    obj.imgDim.forEach((val) => {
      bufferInfo = _serializer.int64(val, bufferInfo);
    });
    // Serialize message field [pixelsPerMeter]
    bufferInfo = _serializer.float64(obj.pixelsPerMeter, bufferInfo);
    // Serialize message field [distanceToChessboard]
    bufferInfo = _serializer.float64(obj.distanceToChessboard, bufferInfo);
    // Serialize message field [chessboardDim]
    obj.chessboardDim.forEach((val) => {
      bufferInfo = _serializer.int64(val, bufferInfo);
    });
    // Serialize message field [squareWidth]
    bufferInfo = _serializer.float64(obj.squareWidth, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type calibrate_imageRequest
    let tmp;
    let len;
    let data = new calibrate_imageRequest();
    // Deserialize message field [image]
    tmp = sensor_msgs.msg.Image.deserialize(buffer);
    data.image = tmp.data;
    buffer = tmp.buffer;
    len = 2;
    // Deserialize message field [imgDim]
    for (let i = 0; i < len; ++i) {
      tmp = _deserializer.int64(buffer);
      data.imgDim[i] = tmp.data;
      buffer = tmp.buffer;
    }
    // Deserialize message field [pixelsPerMeter]
    tmp = _deserializer.float64(buffer);
    data.pixelsPerMeter = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [distanceToChessboard]
    tmp = _deserializer.float64(buffer);
    data.distanceToChessboard = tmp.data;
    buffer = tmp.buffer;
    len = 2;
    // Deserialize message field [chessboardDim]
    for (let i = 0; i < len; ++i) {
      tmp = _deserializer.int64(buffer);
      data.chessboardDim[i] = tmp.data;
      buffer = tmp.buffer;
    }
    // Deserialize message field [squareWidth]
    tmp = _deserializer.float64(buffer);
    data.squareWidth = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'avc/calibrate_imageRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f7709420ba6eef1457180f15a14042fc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/Image image
    int64[2] imgDim
    float64 pixelsPerMeter
    float64 distanceToChessboard
    int64[2] chessboardDim
    float64 squareWidth
    
    ================================================================================
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of cameara
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
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

class calibrate_imageResponse {
  constructor() {
    this.x = false;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type calibrate_imageResponse
    // Serialize message field [x]
    bufferInfo = _serializer.bool(obj.x, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type calibrate_imageResponse
    let tmp;
    let len;
    let data = new calibrate_imageResponse();
    // Deserialize message field [x]
    tmp = _deserializer.bool(buffer);
    data.x = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'avc/calibrate_imageResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6e7c7f7c11db7cc314e9efb95e17c2ed';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool x
    
    `;
  }

};

module.exports = {
  Request: calibrate_imageRequest,
  Response: calibrate_imageResponse
};
