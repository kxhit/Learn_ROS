// Auto-generated. Do not edit!

// (in-package tcp2ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class rtkGPSmessage {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ROS_time = null;
      this.GPS_time = null;
      this.vaild_flag = null;
      this.flash_state = null;
      this.north_meter = null;
      this.east_meter = null;
      this.yaw_rad = null;
    }
    else {
      if (initObj.hasOwnProperty('ROS_time')) {
        this.ROS_time = initObj.ROS_time
      }
      else {
        this.ROS_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('GPS_time')) {
        this.GPS_time = initObj.GPS_time
      }
      else {
        this.GPS_time = '';
      }
      if (initObj.hasOwnProperty('vaild_flag')) {
        this.vaild_flag = initObj.vaild_flag
      }
      else {
        this.vaild_flag = false;
      }
      if (initObj.hasOwnProperty('flash_state')) {
        this.flash_state = initObj.flash_state
      }
      else {
        this.flash_state = '';
      }
      if (initObj.hasOwnProperty('north_meter')) {
        this.north_meter = initObj.north_meter
      }
      else {
        this.north_meter = 0.0;
      }
      if (initObj.hasOwnProperty('east_meter')) {
        this.east_meter = initObj.east_meter
      }
      else {
        this.east_meter = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_rad')) {
        this.yaw_rad = initObj.yaw_rad
      }
      else {
        this.yaw_rad = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type rtkGPSmessage
    // Serialize message field [ROS_time]
    bufferOffset = _serializer.time(obj.ROS_time, buffer, bufferOffset);
    // Serialize message field [GPS_time]
    bufferOffset = _serializer.string(obj.GPS_time, buffer, bufferOffset);
    // Serialize message field [vaild_flag]
    bufferOffset = _serializer.bool(obj.vaild_flag, buffer, bufferOffset);
    // Serialize message field [flash_state]
    bufferOffset = _serializer.string(obj.flash_state, buffer, bufferOffset);
    // Serialize message field [north_meter]
    bufferOffset = _serializer.float64(obj.north_meter, buffer, bufferOffset);
    // Serialize message field [east_meter]
    bufferOffset = _serializer.float64(obj.east_meter, buffer, bufferOffset);
    // Serialize message field [yaw_rad]
    bufferOffset = _serializer.float64(obj.yaw_rad, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type rtkGPSmessage
    let len;
    let data = new rtkGPSmessage(null);
    // Deserialize message field [ROS_time]
    data.ROS_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [GPS_time]
    data.GPS_time = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [vaild_flag]
    data.vaild_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [flash_state]
    data.flash_state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [north_meter]
    data.north_meter = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [east_meter]
    data.east_meter = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw_rad]
    data.yaw_rad = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.GPS_time.length;
    length += object.flash_state.length;
    return length + 41;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tcp2ros/rtkGPSmessage';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '579fa07aed9107c31ed915330a747d64';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time ROS_time
    string GPS_time
    bool vaild_flag
    string flash_state
    float64 north_meter
    float64 east_meter
    float64 yaw_rad
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new rtkGPSmessage(null);
    if (msg.ROS_time !== undefined) {
      resolved.ROS_time = msg.ROS_time;
    }
    else {
      resolved.ROS_time = {secs: 0, nsecs: 0}
    }

    if (msg.GPS_time !== undefined) {
      resolved.GPS_time = msg.GPS_time;
    }
    else {
      resolved.GPS_time = ''
    }

    if (msg.vaild_flag !== undefined) {
      resolved.vaild_flag = msg.vaild_flag;
    }
    else {
      resolved.vaild_flag = false
    }

    if (msg.flash_state !== undefined) {
      resolved.flash_state = msg.flash_state;
    }
    else {
      resolved.flash_state = ''
    }

    if (msg.north_meter !== undefined) {
      resolved.north_meter = msg.north_meter;
    }
    else {
      resolved.north_meter = 0.0
    }

    if (msg.east_meter !== undefined) {
      resolved.east_meter = msg.east_meter;
    }
    else {
      resolved.east_meter = 0.0
    }

    if (msg.yaw_rad !== undefined) {
      resolved.yaw_rad = msg.yaw_rad;
    }
    else {
      resolved.yaw_rad = 0.0
    }

    return resolved;
    }
};

module.exports = rtkGPSmessage;
