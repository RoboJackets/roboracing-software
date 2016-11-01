/*
 *    Copyright 2016 Rethink Robotics
 *
 *    Copyright 2016 Chris Smith
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

'use strict'

//-----------------------------------------------------------------------------
// Base Type Deserializer Functions
//-----------------------------------------------------------------------------

let StringDeserializer = function(buffer) {
  let len = buffer.readUInt32LE(0);
  let newBufStart = len + 4;
  let str = buffer.slice(4, newBufStart).toString();
  buffer = buffer.slice(newBufStart);
  return {
    data: str,
    buffer: buffer
  };
}

let UInt8Deserializer = function(buffer) {
  let val = buffer.readUInt8(0);
  buffer = buffer.slice(1);
  return {
    data: val,
    buffer: buffer
  };
}

let UInt16Deserializer = function(buffer) {
  let val = buffer.readUInt16LE(0);
  buffer = buffer.slice(2);
  return {
    data: val,
    buffer: buffer
  };
}

let UInt32Deserializer = function(buffer) {
  let val = buffer.readUInt32LE(0);
  buffer = buffer.slice(4);
  return {
    data: val,
    buffer: buffer
  };
}

let UInt64Deserializer = function(buffer) {
  // FIXME: best way to do this??
  let val = buffer.slice(0, 8)
  buffer = buffer.slice(8);
  return {
    data: val,
    buffer: buffer
  };
}

let Int8Deserializer = function(buffer) {
  let val = buffer.readInt8(0);
  buffer = buffer.slice(1);
  return {
    data: val,
    buffer: buffer
  };
}

let Int16Deserializer = function(buffer) {
  let val = buffer.readInt16LE(0);
  buffer = buffer.slice(2);
  return {
    data: val,
    buffer: buffer
  };
}

let Int32Deserializer = function(buffer) {
  let val = buffer.readInt32LE(0);
  buffer = buffer.slice(4);
  return {
    data: val,
    buffer: buffer
  };
}

let Int64Deserializer = function(buffer) {
  // FIXME: best way to do this??
  let val = buffer.slice(0, 8)
  buffer = buffer.slice(8);
  return {
    data: val,
    buffer: buffer
  };
}

let Float32Deserializer = function(buffer) {
  let val = buffer.readFloatLE(0);
  buffer = buffer.slice(4);
  return {
    data: val,
    buffer: buffer
  };
}

let Float64Deserializer = function(buffer) {
  let val = buffer.readDoubleLE(0);
  buffer = buffer.slice(8);
  return {
    data: val,
    buffer: buffer
  };
}

let TimeDeserializer = function(buffer) {
  let info = Int32Deserializer(buffer);
  buffer = info.buffer;
  let secs = info.data;
  info = Int32Deserializer(buffer);
  let nsecs = info.data;
  return {
    data: {secs: secs, nsecs: nsecs},
    buffer: info.buffer
  };
}

let BoolDeserializer = function(buffer) {
  let ret = Int8Deserializer(buffer);
  ret.data = !!ret.data;
  return ret;
};

//-----------------------------------------------------------------------------

module.exports = {
  string: StringDeserializer,
  float32: Float32Deserializer,
  float64: Float64Deserializer,
  bool: BoolDeserializer,
  int8: Int8Deserializer,
  int16: Int16Deserializer,
  int32: Int32Deserializer,
  int64: Int64Deserializer,
  uint8: UInt8Deserializer,
  uint16: UInt16Deserializer,
  uint32: UInt32Deserializer,
  uint64: UInt64Deserializer,
  char: UInt8Deserializer,
  byte: Int8Deserializer,
  time: TimeDeserializer,
  duration: TimeDeserializer
};
