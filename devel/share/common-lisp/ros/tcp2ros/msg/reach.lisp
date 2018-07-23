; Auto-generated. Do not edit!


(cl:in-package tcp2ros-msg)


;//! \htmlinclude reach.msg.html

(cl:defclass <reach> (roslisp-msg-protocol:ros-message)
  ((time
    :reader time
    :initarg :time
    :type cl:real
    :initform 0)
   (reach
    :reader reach
    :initarg :reach
    :type cl:integer
    :initform 0))
)

(cl:defclass reach (<reach>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <reach>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'reach)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tcp2ros-msg:<reach> is deprecated: use tcp2ros-msg:reach instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <reach>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:time-val is deprecated.  Use tcp2ros-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'reach-val :lambda-list '(m))
(cl:defmethod reach-val ((m <reach>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:reach-val is deprecated.  Use tcp2ros-msg:reach instead.")
  (reach m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <reach>) ostream)
  "Serializes a message object of type '<reach>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time) (cl:floor (cl:slot-value msg 'time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let* ((signed (cl:slot-value msg 'reach)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <reach>) istream)
  "Deserializes a message object of type '<reach>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reach) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<reach>)))
  "Returns string type for a message object of type '<reach>"
  "tcp2ros/reach")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'reach)))
  "Returns string type for a message object of type 'reach"
  "tcp2ros/reach")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<reach>)))
  "Returns md5sum for a message object of type '<reach>"
  "2dcc348a5da28ada0f2c0d073cc9de9a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'reach)))
  "Returns md5sum for a message object of type 'reach"
  "2dcc348a5da28ada0f2c0d073cc9de9a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<reach>)))
  "Returns full string definition for message of type '<reach>"
  (cl:format cl:nil "time time~%int32 reach~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'reach)))
  "Returns full string definition for message of type 'reach"
  (cl:format cl:nil "time time~%int32 reach~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <reach>))
  (cl:+ 0
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <reach>))
  "Converts a ROS message object to a list"
  (cl:list 'reach
    (cl:cons ':time (time msg))
    (cl:cons ':reach (reach msg))
))
