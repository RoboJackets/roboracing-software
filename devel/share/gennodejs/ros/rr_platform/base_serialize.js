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

let Encoding = 'hex';

//-----------------------------------------------------------------------------
// Base Type Serializer Functions
//-----------------------------------------------------------------------------

let StringSerializer = function(val, bufferInfo) {
  let lenBuf = new Buffer(4);
  let stringBuf = new Buffer(val);
  let len = stringBuf.length;
  lenBuf.writeUInt32LE(len, 0);
  bufferInfo.buffer.push(lenBuf);
  bufferInfo.buffer.push(stringBuf);
  bufferInfo.length += len + 4;
  return bufferInfo;
}

let UInt8Serializer = function(val, bufferInfo) {
  let buf = new Buffer(1);
  buf.writeUInt8(val, 0);
  bufferInfo.buffer.push(buf);
  bufferInfo.length += 1;
  return bufferInfo;
}

let UInt16Serializer = function(val, bufferInfo) {
  let buf = new Buffer(2);
  buf.writeUInt16LE(val, 0);
  bufferInfo.buffer.push(buf);
  bufferInfo.length += 2;
  return bufferInfo;
}

let UInt32Serializer = function(val, bufferInfo) {
  let buf = new Buffer(4);
  buf.writeUInt32LE(val, 0);
  bufferInfo.buffer.push(buf);
  bufferInfo.length += 4;
  return bufferInfo;
}

let UInt64Serializer = function(val, bufferInfo) {
  // FIXME: best way to do this??
  if (!(val instanceof Buffer) || val.length !== 8) {
    throw new Error('Unable to serialize Uint64 - must be 8 byte buffer');
  }
  bufferInfo.buffer.push(val);
  bufferInfo.length += 8;
  return bufferInfo;
}

let Int8Serializer = function(val, bufferInfo) {
  let buf = new Buffer(1);
  buf.writeInt8(val, 0);
  bufferInfo.buffer.push(buf);
  bufferInfo.length += 1;
  return bufferInfo;
}

let Int16Serializer = function(val, bufferInfo) {
  let buf = new Buffer(2);
  buf.writeInt16LE(val, 0);
  bufferInfo.buffer.push(buf);
  bufferInfo.length += 2;
  return bufferInfo;
}

let Int32Serializer = function(val, bufferInfo) {
  let buf = new Buffer(4);
  buf.writeInt32LE(val, 0);
  bufferInfo.buffer.push(buf);
  bufferInfo.length += 4;
  return bufferInfo;
}

let Int64Serializer = function(val, bufferInfo) {
  // FIXME: best way to do this??
  if (!(val instanceof Buffer) || val.length !== 8) {
    throw new Error('Unable to serialize Uint64 - must be 8 byte buffer');
  }
  bufferInfo.buffer.push(val);
  bufferInfo.length += 8;
  return bufferInfo;
}

let Float32Serializer = function(val, bufferInfo) {
  let buf = new Buffer(4);
  buf.writeFloatLE(val, 0);
  bufferInfo.buffer.push(buf);
  bufferInfo.length += 4;
  return bufferInfo;
}

let Float64Serializer = function(val, bufferInfo) {
  let buf = new Buffer(8);
  buf.writeDoubleLE(val, 0);
  bufferInfo.buffer.push(buf);
  bufferInfo.length += 8;
  return bufferInfo;
}

let TimeSerializer = function(val, bufferInfo) {
  let buf = new Buffer(8);
  buf.writeInt32LE(val.secs, 0);
  buf.writeInt32LE(val.nsecs, 4);
  bufferInfo.buffer.push(buf);
  bufferInfo.length += 8;
  return bufferInfo;
}

let BoolSerializer = function(val, bufferInfo) {
  return Int8Serializer(val ? 1 : 0, bufferInfo)
}

//-----------------------------------------------------------------------------

module.exports = {
  string: StringSerializer,
  float32: Float32Serializer,
  float64: Float64Serializer,
  bool: BoolSerializer,
  int8: Int8Serializer,
  int16: Int16Serializer,
  int32: Int32Serializer,
  int64: Int64Serializer,
  uint8: UInt8Serializer,
  uint16: UInt16Serializer,
  uint32: UInt32Serializer,
  uint64: UInt64Serializer,
  char: UInt8Serializer,
  byte: Int8Serializer,
  time: TimeSerializer,
  duration: TimeSerializer
};
