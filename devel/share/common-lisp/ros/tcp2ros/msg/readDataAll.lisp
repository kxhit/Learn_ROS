; Auto-generated. Do not edit!


(cl:in-package tcp2ros-msg)


;//! \htmlinclude readDataAll.msg.html

(cl:defclass <readDataAll> (roslisp-msg-protocol:ros-message)
  ((time
    :reader time
    :initarg :time
    :type cl:real
    :initform 0)
   (odom1
    :reader odom1
    :initarg :odom1
    :type cl:integer
    :initform 0)
   (odom2
    :reader odom2
    :initarg :odom2
    :type cl:integer
    :initform 0)
   (odom3
    :reader odom3
    :initarg :odom3
    :type cl:integer
    :initform 0)
   (odom4
    :reader odom4
    :initarg :odom4
    :type cl:integer
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0)
   (tank_id
    :reader tank_id
    :initarg :tank_id
    :type cl:integer
    :initform 0)
   (track_point_id
    :reader track_point_id
    :initarg :track_point_id
    :type cl:integer
    :initform 0)
   (first_alignment
    :reader first_alignment
    :initarg :first_alignment
    :type cl:integer
    :initform 0)
   (laser_alignment
    :reader laser_alignment
    :initarg :laser_alignment
    :type cl:integer
    :initform 0)
   (distance_alignment
    :reader distance_alignment
    :initarg :distance_alignment
    :type cl:float
    :initform 0.0)
   (Pillar_distance
    :reader Pillar_distance
    :initarg :Pillar_distance
    :type cl:float
    :initform 0.0)
   (pause
    :reader pause
    :initarg :pause
    :type cl:integer
    :initform 0)
   (stop
    :reader stop
    :initarg :stop
    :type cl:integer
    :initform 0)
   (back_home
    :reader back_home
    :initarg :back_home
    :type cl:integer
    :initform 0)
   (other_car_x
    :reader other_car_x
    :initarg :other_car_x
    :type cl:float
    :initform 0.0)
   (other_car_y
    :reader other_car_y
    :initarg :other_car_y
    :type cl:float
    :initform 0.0)
   (other_car_theta
    :reader other_car_theta
    :initarg :other_car_theta
    :type cl:float
    :initform 0.0)
   (infrared_right
    :reader infrared_right
    :initarg :infrared_right
    :type cl:float
    :initform 0.0)
   (infrared_left
    :reader infrared_left
    :initarg :infrared_left
    :type cl:float
    :initform 0.0)
   (is_start_camera
    :reader is_start_camera
    :initarg :is_start_camera
    :type cl:integer
    :initform 0)
   (next_target_num
    :reader next_target_num
    :initarg :next_target_num
    :type cl:integer
    :initform 0)
   (control1
    :reader control1
    :initarg :control1
    :type cl:integer
    :initform 0)
   (control2
    :reader control2
    :initarg :control2
    :type cl:integer
    :initform 0)
   (control3
    :reader control3
    :initarg :control3
    :type cl:integer
    :initform 0)
   (control4
    :reader control4
    :initarg :control4
    :type cl:integer
    :initform 0))
)

(cl:defclass readDataAll (<readDataAll>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <readDataAll>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'readDataAll)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tcp2ros-msg:<readDataAll> is deprecated: use tcp2ros-msg:readDataAll instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:time-val is deprecated.  Use tcp2ros-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'odom1-val :lambda-list '(m))
(cl:defmethod odom1-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:odom1-val is deprecated.  Use tcp2ros-msg:odom1 instead.")
  (odom1 m))

(cl:ensure-generic-function 'odom2-val :lambda-list '(m))
(cl:defmethod odom2-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:odom2-val is deprecated.  Use tcp2ros-msg:odom2 instead.")
  (odom2 m))

(cl:ensure-generic-function 'odom3-val :lambda-list '(m))
(cl:defmethod odom3-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:odom3-val is deprecated.  Use tcp2ros-msg:odom3 instead.")
  (odom3 m))

(cl:ensure-generic-function 'odom4-val :lambda-list '(m))
(cl:defmethod odom4-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:odom4-val is deprecated.  Use tcp2ros-msg:odom4 instead.")
  (odom4 m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:mode-val is deprecated.  Use tcp2ros-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'tank_id-val :lambda-list '(m))
(cl:defmethod tank_id-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:tank_id-val is deprecated.  Use tcp2ros-msg:tank_id instead.")
  (tank_id m))

(cl:ensure-generic-function 'track_point_id-val :lambda-list '(m))
(cl:defmethod track_point_id-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:track_point_id-val is deprecated.  Use tcp2ros-msg:track_point_id instead.")
  (track_point_id m))

(cl:ensure-generic-function 'first_alignment-val :lambda-list '(m))
(cl:defmethod first_alignment-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:first_alignment-val is deprecated.  Use tcp2ros-msg:first_alignment instead.")
  (first_alignment m))

(cl:ensure-generic-function 'laser_alignment-val :lambda-list '(m))
(cl:defmethod laser_alignment-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:laser_alignment-val is deprecated.  Use tcp2ros-msg:laser_alignment instead.")
  (laser_alignment m))

(cl:ensure-generic-function 'distance_alignment-val :lambda-list '(m))
(cl:defmethod distance_alignment-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:distance_alignment-val is deprecated.  Use tcp2ros-msg:distance_alignment instead.")
  (distance_alignment m))

(cl:ensure-generic-function 'Pillar_distance-val :lambda-list '(m))
(cl:defmethod Pillar_distance-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:Pillar_distance-val is deprecated.  Use tcp2ros-msg:Pillar_distance instead.")
  (Pillar_distance m))

(cl:ensure-generic-function 'pause-val :lambda-list '(m))
(cl:defmethod pause-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:pause-val is deprecated.  Use tcp2ros-msg:pause instead.")
  (pause m))

(cl:ensure-generic-function 'stop-val :lambda-list '(m))
(cl:defmethod stop-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:stop-val is deprecated.  Use tcp2ros-msg:stop instead.")
  (stop m))

(cl:ensure-generic-function 'back_home-val :lambda-list '(m))
(cl:defmethod back_home-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:back_home-val is deprecated.  Use tcp2ros-msg:back_home instead.")
  (back_home m))

(cl:ensure-generic-function 'other_car_x-val :lambda-list '(m))
(cl:defmethod other_car_x-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:other_car_x-val is deprecated.  Use tcp2ros-msg:other_car_x instead.")
  (other_car_x m))

(cl:ensure-generic-function 'other_car_y-val :lambda-list '(m))
(cl:defmethod other_car_y-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:other_car_y-val is deprecated.  Use tcp2ros-msg:other_car_y instead.")
  (other_car_y m))

(cl:ensure-generic-function 'other_car_theta-val :lambda-list '(m))
(cl:defmethod other_car_theta-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:other_car_theta-val is deprecated.  Use tcp2ros-msg:other_car_theta instead.")
  (other_car_theta m))

(cl:ensure-generic-function 'infrared_right-val :lambda-list '(m))
(cl:defmethod infrared_right-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:infrared_right-val is deprecated.  Use tcp2ros-msg:infrared_right instead.")
  (infrared_right m))

(cl:ensure-generic-function 'infrared_left-val :lambda-list '(m))
(cl:defmethod infrared_left-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:infrared_left-val is deprecated.  Use tcp2ros-msg:infrared_left instead.")
  (infrared_left m))

(cl:ensure-generic-function 'is_start_camera-val :lambda-list '(m))
(cl:defmethod is_start_camera-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:is_start_camera-val is deprecated.  Use tcp2ros-msg:is_start_camera instead.")
  (is_start_camera m))

(cl:ensure-generic-function 'next_target_num-val :lambda-list '(m))
(cl:defmethod next_target_num-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:next_target_num-val is deprecated.  Use tcp2ros-msg:next_target_num instead.")
  (next_target_num m))

(cl:ensure-generic-function 'control1-val :lambda-list '(m))
(cl:defmethod control1-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:control1-val is deprecated.  Use tcp2ros-msg:control1 instead.")
  (control1 m))

(cl:ensure-generic-function 'control2-val :lambda-list '(m))
(cl:defmethod control2-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:control2-val is deprecated.  Use tcp2ros-msg:control2 instead.")
  (control2 m))

(cl:ensure-generic-function 'control3-val :lambda-list '(m))
(cl:defmethod control3-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:control3-val is deprecated.  Use tcp2ros-msg:control3 instead.")
  (control3 m))

(cl:ensure-generic-function 'control4-val :lambda-list '(m))
(cl:defmethod control4-val ((m <readDataAll>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tcp2ros-msg:control4-val is deprecated.  Use tcp2ros-msg:control4 instead.")
  (control4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <readDataAll>) ostream)
  "Serializes a message object of type '<readDataAll>"
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
  (cl:let* ((signed (cl:slot-value msg 'odom1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'odom2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'odom3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'odom4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tank_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'track_point_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'first_alignment)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'laser_alignment)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance_alignment))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Pillar_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'pause)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'stop)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'back_home)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'other_car_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'other_car_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'other_car_theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'infrared_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'infrared_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'is_start_camera)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'next_target_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'control1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'control2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'control3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'control4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <readDataAll>) istream)
  "Deserializes a message object of type '<readDataAll>"
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
      (cl:setf (cl:slot-value msg 'odom1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'odom2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'odom3) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'odom4) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tank_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'track_point_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'first_alignment) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'laser_alignment) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance_alignment) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Pillar_distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pause) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stop) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'back_home) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'other_car_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'other_car_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'other_car_theta) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'infrared_right) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'infrared_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'is_start_camera) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'next_target_num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'control1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'control2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'control3) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'control4) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<readDataAll>)))
  "Returns string type for a message object of type '<readDataAll>"
  "tcp2ros/readDataAll")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'readDataAll)))
  "Returns string type for a message object of type 'readDataAll"
  "tcp2ros/readDataAll")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<readDataAll>)))
  "Returns md5sum for a message object of type '<readDataAll>"
  "965a724e29296364637aadf1b2f664ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'readDataAll)))
  "Returns md5sum for a message object of type 'readDataAll"
  "965a724e29296364637aadf1b2f664ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<readDataAll>)))
  "Returns full string definition for message of type '<readDataAll>"
  (cl:format cl:nil "time time~%int32 odom1~%int32 odom2~%int32 odom3~%int32 odom4~%int32 mode~%int32 tank_id~%int32 track_point_id~%int32 first_alignment~%int32 laser_alignment~%float32 distance_alignment~%float32 Pillar_distance~%int32 pause~%int32 stop~%int32 back_home~%float32 other_car_x~%float32 other_car_y~%float32 other_car_theta~%float32 infrared_right~%float32 infrared_left~%int32 is_start_camera~%int32 next_target_num~%int32 control1~%int32 control2~%int32 control3~%int32 control4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'readDataAll)))
  "Returns full string definition for message of type 'readDataAll"
  (cl:format cl:nil "time time~%int32 odom1~%int32 odom2~%int32 odom3~%int32 odom4~%int32 mode~%int32 tank_id~%int32 track_point_id~%int32 first_alignment~%int32 laser_alignment~%float32 distance_alignment~%float32 Pillar_distance~%int32 pause~%int32 stop~%int32 back_home~%float32 other_car_x~%float32 other_car_y~%float32 other_car_theta~%float32 infrared_right~%float32 infrared_left~%int32 is_start_camera~%int32 next_target_num~%int32 control1~%int32 control2~%int32 control3~%int32 control4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <readDataAll>))
  (cl:+ 0
     8
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <readDataAll>))
  "Converts a ROS message object to a list"
  (cl:list 'readDataAll
    (cl:cons ':time (time msg))
    (cl:cons ':odom1 (odom1 msg))
    (cl:cons ':odom2 (odom2 msg))
    (cl:cons ':odom3 (odom3 msg))
    (cl:cons ':odom4 (odom4 msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':tank_id (tank_id msg))
    (cl:cons ':track_point_id (track_point_id msg))
    (cl:cons ':first_alignment (first_alignment msg))
    (cl:cons ':laser_alignment (laser_alignment msg))
    (cl:cons ':distance_alignment (distance_alignment msg))
    (cl:cons ':Pillar_distance (Pillar_distance msg))
    (cl:cons ':pause (pause msg))
    (cl:cons ':stop (stop msg))
    (cl:cons ':back_home (back_home msg))
    (cl:cons ':other_car_x (other_car_x msg))
    (cl:cons ':other_car_y (other_car_y msg))
    (cl:cons ':other_car_theta (other_car_theta msg))
    (cl:cons ':infrared_right (infrared_right msg))
    (cl:cons ':infrared_left (infrared_left msg))
    (cl:cons ':is_start_camera (is_start_camera msg))
    (cl:cons ':next_target_num (next_target_num msg))
    (cl:cons ':control1 (control1 msg))
    (cl:cons ':control2 (control2 msg))
    (cl:cons ':control3 (control3 msg))
    (cl:cons ':control4 (control4 msg))
))
