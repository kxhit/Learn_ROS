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

class readDataAll {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.time = null;
      this.odom1 = null;
      this.odom2 = null;
      this.odom3 = null;
      this.odom4 = null;
      this.mode = null;
      this.tank_id = null;
      this.track_point_id = null;
      this.first_alignment = null;
      this.laser_alignment = null;
      this.distance_alignment = null;
      this.Pillar_distance = null;
      this.pause = null;
      this.stop = null;
      this.back_home = null;
      this.other_car_x = null;
      this.other_car_y = null;
      this.other_car_theta = null;
      this.infrared_right = null;
      this.infrared_left = null;
      this.is_start_camera = null;
      this.next_target_num = null;
      this.control1 = null;
      this.control2 = null;
      this.control3 = null;
      this.control4 = null;
    }
    else {
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('odom1')) {
        this.odom1 = initObj.odom1
      }
      else {
        this.odom1 = 0;
      }
      if (initObj.hasOwnProperty('odom2')) {
        this.odom2 = initObj.odom2
      }
      else {
        this.odom2 = 0;
      }
      if (initObj.hasOwnProperty('odom3')) {
        this.odom3 = initObj.odom3
      }
      else {
        this.odom3 = 0;
      }
      if (initObj.hasOwnProperty('odom4')) {
        this.odom4 = initObj.odom4
      }
      else {
        this.odom4 = 0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('tank_id')) {
        this.tank_id = initObj.tank_id
      }
      else {
        this.tank_id = 0;
      }
      if (initObj.hasOwnProperty('track_point_id')) {
        this.track_point_id = initObj.track_point_id
      }
      else {
        this.track_point_id = 0;
      }
      if (initObj.hasOwnProperty('first_alignment')) {
        this.first_alignment = initObj.first_alignment
      }
      else {
        this.first_alignment = 0;
      }
      if (initObj.hasOwnProperty('laser_alignment')) {
        this.laser_alignment = initObj.laser_alignment
      }
      else {
        this.laser_alignment = 0;
      }
      if (initObj.hasOwnProperty('distance_alignment')) {
        this.distance_alignment = initObj.distance_alignment
      }
      else {
        this.distance_alignment = 0.0;
      }
      if (initObj.hasOwnProperty('Pillar_distance')) {
        this.Pillar_distance = initObj.Pillar_distance
      }
      else {
        this.Pillar_distance = 0.0;
      }
      if (initObj.hasOwnProperty('pause')) {
        this.pause = initObj.pause
      }
      else {
        this.pause = 0;
      }
      if (initObj.hasOwnProperty('stop')) {
        this.stop = initObj.stop
      }
      else {
        this.stop = 0;
      }
      if (initObj.hasOwnProperty('back_home')) {
        this.back_home = initObj.back_home
      }
      else {
        this.back_home = 0;
      }
      if (initObj.hasOwnProperty('other_car_x')) {
        this.other_car_x = initObj.other_car_x
      }
      else {
        this.other_car_x = 0.0;
      }
      if (initObj.hasOwnProperty('other_car_y')) {
        this.other_car_y = initObj.other_car_y
      }
      else {
        this.other_car_y = 0.0;
      }
      if (initObj.hasOwnProperty('other_car_theta')) {
        this.other_car_theta = initObj.other_car_theta
      }
      else {
        this.other_car_theta = 0.0;
      }
      if (initObj.hasOwnProperty('infrared_right')) {
        this.infrared_right = initObj.infrared_right
      }
      else {
        this.infrared_right = 0.0;
      }
      if (initObj.hasOwnProperty('infrared_left')) {
        this.infrared_left = initObj.infrared_left
      }
      else {
        this.infrared_left = 0.0;
      }
      if (initObj.hasOwnProperty('is_start_camera')) {
        this.is_start_camera = initObj.is_start_camera
      }
      else {
        this.is_start_camera = 0;
      }
      if (initObj.hasOwnProperty('next_target_num')) {
        this.next_target_num = initObj.next_target_num
      }
      else {
        this.next_target_num = 0;
      }
      if (initObj.hasOwnProperty('control1')) {
        this.control1 = initObj.control1
      }
      else {
        this.control1 = 0;
      }
      if (initObj.hasOwnProperty('control2')) {
        this.control2 = initObj.control2
      }
      else {
        this.control2 = 0;
      }
      if (initObj.hasOwnProperty('control3')) {
        this.control3 = initObj.control3
      }
      else {
        this.control3 = 0;
      }
      if (initObj.hasOwnProperty('control4')) {
        this.control4 = initObj.control4
      }
      else {
        this.control4 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type readDataAll
    // Serialize message field [time]
    bufferOffset = _serializer.time(obj.time, buffer, bufferOffset);
    // Serialize message field [odom1]
    bufferOffset = _serializer.int32(obj.odom1, buffer, bufferOffset);
    // Serialize message field [odom2]
    bufferOffset = _serializer.int32(obj.odom2, buffer, bufferOffset);
    // Serialize message field [odom3]
    bufferOffset = _serializer.int32(obj.odom3, buffer, bufferOffset);
    // Serialize message field [odom4]
    bufferOffset = _serializer.int32(obj.odom4, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.int32(obj.mode, buffer, bufferOffset);
    // Serialize message field [tank_id]
    bufferOffset = _serializer.int32(obj.tank_id, buffer, bufferOffset);
    // Serialize message field [track_point_id]
    bufferOffset = _serializer.int32(obj.track_point_id, buffer, bufferOffset);
    // Serialize message field [first_alignment]
    bufferOffset = _serializer.int32(obj.first_alignment, buffer, bufferOffset);
    // Serialize message field [laser_alignment]
    bufferOffset = _serializer.int32(obj.laser_alignment, buffer, bufferOffset);
    // Serialize message field [distance_alignment]
    bufferOffset = _serializer.float32(obj.distance_alignment, buffer, bufferOffset);
    // Serialize message field [Pillar_distance]
    bufferOffset = _serializer.float32(obj.Pillar_distance, buffer, bufferOffset);
    // Serialize message field [pause]
    bufferOffset = _serializer.int32(obj.pause, buffer, bufferOffset);
    // Serialize message field [stop]
    bufferOffset = _serializer.int32(obj.stop, buffer, bufferOffset);
    // Serialize message field [back_home]
    bufferOffset = _serializer.int32(obj.back_home, buffer, bufferOffset);
    // Serialize message field [other_car_x]
    bufferOffset = _serializer.float32(obj.other_car_x, buffer, bufferOffset);
    // Serialize message field [other_car_y]
    bufferOffset = _serializer.float32(obj.other_car_y, buffer, bufferOffset);
    // Serialize message field [other_car_theta]
    bufferOffset = _serializer.float32(obj.other_car_theta, buffer, bufferOffset);
    // Serialize message field [infrared_right]
    bufferOffset = _serializer.float32(obj.infrared_right, buffer, bufferOffset);
    // Serialize message field [infrared_left]
    bufferOffset = _serializer.float32(obj.infrared_left, buffer, bufferOffset);
    // Serialize message field [is_start_camera]
    bufferOffset = _serializer.int32(obj.is_start_camera, buffer, bufferOffset);
    // Serialize message field [next_target_num]
    bufferOffset = _serializer.int32(obj.next_target_num, buffer, bufferOffset);
    // Serialize message field [control1]
    bufferOffset = _serializer.int32(obj.control1, buffer, bufferOffset);
    // Serialize message field [control2]
    bufferOffset = _serializer.int32(obj.control2, buffer, bufferOffset);
    // Serialize message field [control3]
    bufferOffset = _serializer.int32(obj.control3, buffer, bufferOffset);
    // Serialize message field [control4]
    bufferOffset = _serializer.int32(obj.control4, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type readDataAll
    let len;
    let data = new readDataAll(null);
    // Deserialize message field [time]
    data.time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [odom1]
    data.odom1 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [odom2]
    data.odom2 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [odom3]
    data.odom3 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [odom4]
    data.odom4 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [tank_id]
    data.tank_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [track_point_id]
    data.track_point_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [first_alignment]
    data.first_alignment = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [laser_alignment]
    data.laser_alignment = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [distance_alignment]
    data.distance_alignment = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Pillar_distance]
    data.Pillar_distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pause]
    data.pause = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [stop]
    data.stop = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [back_home]
    data.back_home = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [other_car_x]
    data.other_car_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [other_car_y]
    data.other_car_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [other_car_theta]
    data.other_car_theta = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [infrared_right]
    data.infrared_right = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [infrared_left]
    data.infrared_left = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [is_start_camera]
    data.is_start_camera = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [next_target_num]
    data.next_target_num = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [control1]
    data.control1 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [control2]
    data.control2 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [control3]
    data.control3 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [control4]
    data.control4 = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 108;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tcp2ros/readDataAll';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '965a724e29296364637aadf1b2f664ed';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time time
    int32 odom1
    int32 odom2
    int32 odom3
    int32 odom4
    int32 mode
    int32 tank_id
    int32 track_point_id
    int32 first_alignment
    int32 laser_alignment
    float32 distance_alignment
    float32 Pillar_distance
    int32 pause
    int32 stop
    int32 back_home
    float32 other_car_x
    float32 other_car_y
    float32 other_car_theta
    float32 infrared_right
    float32 infrared_left
    int32 is_start_camera
    int32 next_target_num
    int32 control1
    int32 control2
    int32 control3
    int32 control4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new readDataAll(null);
    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = {secs: 0, nsecs: 0}
    }

    if (msg.odom1 !== undefined) {
      resolved.odom1 = msg.odom1;
    }
    else {
      resolved.odom1 = 0
    }

    if (msg.odom2 !== undefined) {
      resolved.odom2 = msg.odom2;
    }
    else {
      resolved.odom2 = 0
    }

    if (msg.odom3 !== undefined) {
      resolved.odom3 = msg.odom3;
    }
    else {
      resolved.odom3 = 0
    }

    if (msg.odom4 !== undefined) {
      resolved.odom4 = msg.odom4;
    }
    else {
      resolved.odom4 = 0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.tank_id !== undefined) {
      resolved.tank_id = msg.tank_id;
    }
    else {
      resolved.tank_id = 0
    }

    if (msg.track_point_id !== undefined) {
      resolved.track_point_id = msg.track_point_id;
    }
    else {
      resolved.track_point_id = 0
    }

    if (msg.first_alignment !== undefined) {
      resolved.first_alignment = msg.first_alignment;
    }
    else {
      resolved.first_alignment = 0
    }

    if (msg.laser_alignment !== undefined) {
      resolved.laser_alignment = msg.laser_alignment;
    }
    else {
      resolved.laser_alignment = 0
    }

    if (msg.distance_alignment !== undefined) {
      resolved.distance_alignment = msg.distance_alignment;
    }
    else {
      resolved.distance_alignment = 0.0
    }

    if (msg.Pillar_distance !== undefined) {
      resolved.Pillar_distance = msg.Pillar_distance;
    }
    else {
      resolved.Pillar_distance = 0.0
    }

    if (msg.pause !== undefined) {
      resolved.pause = msg.pause;
    }
    else {
      resolved.pause = 0
    }

    if (msg.stop !== undefined) {
      resolved.stop = msg.stop;
    }
    else {
      resolved.stop = 0
    }

    if (msg.back_home !== undefined) {
      resolved.back_home = msg.back_home;
    }
    else {
      resolved.back_home = 0
    }

    if (msg.other_car_x !== undefined) {
      resolved.other_car_x = msg.other_car_x;
    }
    else {
      resolved.other_car_x = 0.0
    }

    if (msg.other_car_y !== undefined) {
      resolved.other_car_y = msg.other_car_y;
    }
    else {
      resolved.other_car_y = 0.0
    }

    if (msg.other_car_theta !== undefined) {
      resolved.other_car_theta = msg.other_car_theta;
    }
    else {
      resolved.other_car_theta = 0.0
    }

    if (msg.infrared_right !== undefined) {
      resolved.infrared_right = msg.infrared_right;
    }
    else {
      resolved.infrared_right = 0.0
    }

    if (msg.infrared_left !== undefined) {
      resolved.infrared_left = msg.infrared_left;
    }
    else {
      resolved.infrared_left = 0.0
    }

    if (msg.is_start_camera !== undefined) {
      resolved.is_start_camera = msg.is_start_camera;
    }
    else {
      resolved.is_start_camera = 0
    }

    if (msg.next_target_num !== undefined) {
      resolved.next_target_num = msg.next_target_num;
    }
    else {
      resolved.next_target_num = 0
    }

    if (msg.control1 !== undefined) {
      resolved.control1 = msg.control1;
    }
    else {
      resolved.control1 = 0
    }

    if (msg.control2 !== undefined) {
      resolved.control2 = msg.control2;
    }
    else {
      resolved.control2 = 0
    }

    if (msg.control3 !== undefined) {
      resolved.control3 = msg.control3;
    }
    else {
      resolved.control3 = 0
    }

    if (msg.control4 !== undefined) {
      resolved.control4 = msg.control4;
    }
    else {
      resolved.control4 = 0
    }

    return resolved;
    }
};

module.exports = readDataAll;
