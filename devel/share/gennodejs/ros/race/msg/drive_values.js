// Auto-generated. Do not edit!

// (in-package race.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class drive_values {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.throttle = null;
      this.steering = null;
    }
    else {
      if (initObj.hasOwnProperty('throttle')) {
        this.throttle = initObj.throttle
      }
      else {
        this.throttle = 0;
      }
      if (initObj.hasOwnProperty('steering')) {
        this.steering = initObj.steering
      }
      else {
        this.steering = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type drive_values
    // Serialize message field [throttle]
    bufferOffset = _serializer.int16(obj.throttle, buffer, bufferOffset);
    // Serialize message field [steering]
    bufferOffset = _serializer.int16(obj.steering, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type drive_values
    let len;
    let data = new drive_values(null);
    // Deserialize message field [throttle]
    data.throttle = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [steering]
    data.steering = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'race/drive_values';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0194e179a126494f7021e4f23090fd6b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 throttle
    int16 steering
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new drive_values(null);
    if (msg.throttle !== undefined) {
      resolved.throttle = msg.throttle;
    }
    else {
      resolved.throttle = 0
    }

    if (msg.steering !== undefined) {
      resolved.steering = msg.steering;
    }
    else {
      resolved.steering = 0
    }

    return resolved;
    }
};

module.exports = drive_values;
