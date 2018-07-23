; Auto-generated. Do not edit!


(cl:in-package tcp2ros-msg)


;//! \htmlinclude rtkGPSmessage.msg.html

(cl:defclass <rtkGPSmessage> (roslisp-msg-protocol:ros-message)
  ((ROS_time
    :reader ROS_time
    :initarg :ROS_time
    :type cl:real
    :initform 0)
   (GPS_time
    :reader GPS_time
    :initarg :GPS_time
    :type cl:string
    :initform "")
   (vaild_flag
    :reader vaild_flag
    :initarg :vaild_flag
    :type cl:boolean
    :initform cl:nil)
   (flash_state
    :reader flash_state
    :initarg :flash_state
    :type cl:string
    :initform "")
   (north_meter
    :reader north_meter
    :initarg :north_meter
    :type cl:float
    :initform 0.0)
   (east_meter
    :reader east_meter
    :initarg :east_meter
    :type cl:float
    :initform 0.0)
   (yaw_rad
    :reader yaw_rad
    :initarg :yaw_rad
    :type cl:float
    :initform 0.0))
)

(cl:defclass rtkGPSmessage (<rtkGPSmessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rtkGPSmessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rtkGPSmessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tcp2ros-msg:<rtkGPSmessage> is deprecated: use tcp2ros-msg:rtkGPSmessage instead.")))

(cl:ensure-generic-function 'ROS_time-val :lambda-list '(m))
(cl:defmethod ROS_time-val ((m <rtkGPSmessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:ROS_time-val is deprecated.  Use tcp2ros-msg:ROS_time instead.")
  (ROS_time m))

(cl:ensure-generic-function 'GPS_time-val :lambda-list '(m))
(cl:defmethod GPS_time-val ((m <rtkGPSmessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:GPS_time-val is deprecated.  Use tcp2ros-msg:GPS_time instead.")
  (GPS_time m))

(cl:ensure-generic-function 'vaild_flag-val :lambda-list '(m))
(cl:defmethod vaild_flag-val ((m <rtkGPSmessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:vaild_flag-val is deprecated.  Use tcp2ros-msg:vaild_flag instead.")
  (vaild_flag m))

(cl:ensure-generic-function 'flash_state-val :lambda-list '(m))
(cl:defmethod flash_state-val ((m <rtkGPSmessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:flash_state-val is deprecated.  Use tcp2ros-msg:flash_state instead.")
  (flash_state m))

(cl:ensure-generic-function 'north_meter-val :lambda-list '(m))
(cl:defmethod north_meter-val ((m <rtkGPSmessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:north_meter-val is deprecated.  Use tcp2ros-msg:north_meter instead.")
  (north_meter m))

(cl:ensure-generic-function 'east_meter-val :lambda-list '(m))
(cl:defmethod east_meter-val ((m <rtkGPSmessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:east_meter-val is deprecated.  Use tcp2ros-msg:east_meter instead.")
  (east_meter m))

(cl:ensure-generic-function 'yaw_rad-val :lambda-list '(m))
(cl:defmethod yaw_rad-val ((m <rtkGPSmessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:yaw_rad-val is deprecated.  Use tcp2ros-msg:yaw_rad instead.")
  (yaw_rad m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rtkGPSmessage>) ostream)
  "Serializes a message object of type '<rtkGPSmessage>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'ROS_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'ROS_time) (cl:floor (cl:slot-value msg 'ROS_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'GPS_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'GPS_time))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'vaild_flag) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'flash_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'flash_state))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'north_meter))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'east_meter))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw_rad))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rtkGPSmessage>) istream)
  "Deserializes a message object of type '<rtkGPSmessage>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ROS_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'GPS_time) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'GPS_time) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'vaild_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'flash_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'flash_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'north_meter) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'east_meter) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_rad) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rtkGPSmessage>)))
  "Returns string type for a message object of type '<rtkGPSmessage>"
  "tcp2ros/rtkGPSmessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rtkGPSmessage)))
  "Returns string type for a message object of type 'rtkGPSmessage"
  "tcp2ros/rtkGPSmessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rtkGPSmessage>)))
  "Returns md5sum for a message object of type '<rtkGPSmessage>"
  "579fa07aed9107c31ed915330a747d64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rtkGPSmessage)))
  "Returns md5sum for a message object of type 'rtkGPSmessage"
  "579fa07aed9107c31ed915330a747d64")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rtkGPSmessage>)))
  "Returns full string definition for message of type '<rtkGPSmessage>"
  (cl:format cl:nil "time ROS_time~%string GPS_time~%bool vaild_flag~%string flash_state~%float64 north_meter~%float64 east_meter~%float64 yaw_rad~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rtkGPSmessage)))
  "Returns full string definition for message of type 'rtkGPSmessage"
  (cl:format cl:nil "time ROS_time~%string GPS_time~%bool vaild_flag~%string flash_state~%float64 north_meter~%float64 east_meter~%float64 yaw_rad~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rtkGPSmessage>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'GPS_time))
     1
     4 (cl:length (cl:slot-value msg 'flash_state))
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rtkGPSmessage>))
  "Converts a ROS message object to a list"
  (cl:list 'rtkGPSmessage
    (cl:cons ':ROS_time (ROS_time msg))
    (cl:cons ':GPS_time (GPS_time msg))
    (cl:cons ':vaild_flag (vaild_flag msg))
    (cl:cons ':flash_state (flash_state msg))
    (cl:cons ':north_meter (north_meter msg))
    (cl:cons ':east_meter (east_meter msg))
    (cl:cons ':yaw_rad (yaw_rad msg))
))
