;; Auto-generated. Do not edit!


(when (boundp 'tcp2ros::readDataAll)
  (if (not (find-package "TCP2ROS"))
    (make-package "TCP2ROS"))
  (shadow 'readDataAll (find-package "TCP2ROS")))
(unless (find-package "TCP2ROS::READDATAALL")
  (make-package "TCP2ROS::READDATAALL"))

(in-package "ROS")
;;//! \htmlinclude readDataAll.msg.html


(defclass tcp2ros::readDataAll
  :super ros::object
  :slots (_time _odom1 _odom2 _odom3 _odom4 _mode _tank_id _track_point_id _first_alignment _laser_alignment _distance_alignment _Pillar_distance _pause _stop _back_home _other_car_x _other_car_y _other_car_theta _infrared_right _infrared_left _is_start_camera _next_target_num _control1 _control2 _control3 _control4 ))

(defmethod tcp2ros::readDataAll
  (:init
   (&key
    ((:time __time) (instance ros::time :init))
    ((:odom1 __odom1) 0)
    ((:odom2 __odom2) 0)
    ((:odom3 __odom3) 0)
    ((:odom4 __odom4) 0)
    ((:mode __mode) 0)
    ((:tank_id __tank_id) 0)
    ((:track_point_id __track_point_id) 0)
    ((:first_alignment __first_alignment) 0)
    ((:laser_alignment __laser_alignment) 0)
    ((:distance_alignment __distance_alignment) 0.0)
    ((:Pillar_distance __Pillar_distance) 0.0)
    ((:pause __pause) 0)
    ((:stop __stop) 0)
    ((:back_home __back_home) 0)
    ((:other_car_x __other_car_x) 0.0)
    ((:other_car_y __other_car_y) 0.0)
    ((:other_car_theta __other_car_theta) 0.0)
    ((:infrared_right __infrared_right) 0.0)
    ((:infrared_left __infrared_left) 0.0)
    ((:is_start_camera __is_start_camera) 0)
    ((:next_target_num __next_target_num) 0)
    ((:control1 __control1) 0)
    ((:control2 __control2) 0)
    ((:control3 __control3) 0)
    ((:control4 __control4) 0)
    )
   (send-super :init)
   (setq _time __time)
   (setq _odom1 (round __odom1))
   (setq _odom2 (round __odom2))
   (setq _odom3 (round __odom3))
   (setq _odom4 (round __odom4))
   (setq _mode (round __mode))
   (setq _tank_id (round __tank_id))
   (setq _track_point_id (round __track_point_id))
   (setq _first_alignment (round __first_alignment))
   (setq _laser_alignment (round __laser_alignment))
   (setq _distance_alignment (float __distance_alignment))
   (setq _Pillar_distance (float __Pillar_distance))
   (setq _pause (round __pause))
   (setq _stop (round __stop))
   (setq _back_home (round __back_home))
   (setq _other_car_x (float __other_car_x))
   (setq _other_car_y (float __other_car_y))
   (setq _other_car_theta (float __other_car_theta))
   (setq _infrared_right (float __infrared_right))
   (setq _infrared_left (float __infrared_left))
   (setq _is_start_camera (round __is_start_camera))
   (setq _next_target_num (round __next_target_num))
   (setq _control1 (round __control1))
   (setq _control2 (round __control2))
   (setq _control3 (round __control3))
   (setq _control4 (round __control4))
   self)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:odom1
   (&optional __odom1)
   (if __odom1 (setq _odom1 __odom1)) _odom1)
  (:odom2
   (&optional __odom2)
   (if __odom2 (setq _odom2 __odom2)) _odom2)
  (:odom3
   (&optional __odom3)
   (if __odom3 (setq _odom3 __odom3)) _odom3)
  (:odom4
   (&optional __odom4)
   (if __odom4 (setq _odom4 __odom4)) _odom4)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:tank_id
   (&optional __tank_id)
   (if __tank_id (setq _tank_id __tank_id)) _tank_id)
  (:track_point_id
   (&optional __track_point_id)
   (if __track_point_id (setq _track_point_id __track_point_id)) _track_point_id)
  (:first_alignment
   (&optional __first_alignment)
   (if __first_alignment (setq _first_alignment __first_alignment)) _first_alignment)
  (:laser_alignment
   (&optional __laser_alignment)
   (if __laser_alignment (setq _laser_alignment __laser_alignment)) _laser_alignment)
  (:distance_alignment
   (&optional __distance_alignment)
   (if __distance_alignment (setq _distance_alignment __distance_alignment)) _distance_alignment)
  (:Pillar_distance
   (&optional __Pillar_distance)
   (if __Pillar_distance (setq _Pillar_distance __Pillar_distance)) _Pillar_distance)
  (:pause
   (&optional __pause)
   (if __pause (setq _pause __pause)) _pause)
  (:stop
   (&optional __stop)
   (if __stop (setq _stop __stop)) _stop)
  (:back_home
   (&optional __back_home)
   (if __back_home (setq _back_home __back_home)) _back_home)
  (:other_car_x
   (&optional __other_car_x)
   (if __other_car_x (setq _other_car_x __other_car_x)) _other_car_x)
  (:other_car_y
   (&optional __other_car_y)
   (if __other_car_y (setq _other_car_y __other_car_y)) _other_car_y)
  (:other_car_theta
   (&optional __other_car_theta)
   (if __other_car_theta (setq _other_car_theta __other_car_theta)) _other_car_theta)
  (:infrared_right
   (&optional __infrared_right)
   (if __infrared_right (setq _infrared_right __infrared_right)) _infrared_right)
  (:infrared_left
   (&optional __infrared_left)
   (if __infrared_left (setq _infrared_left __infrared_left)) _infrared_left)
  (:is_start_camera
   (&optional __is_start_camera)
   (if __is_start_camera (setq _is_start_camera __is_start_camera)) _is_start_camera)
  (:next_target_num
   (&optional __next_target_num)
   (if __next_target_num (setq _next_target_num __next_target_num)) _next_target_num)
  (:control1
   (&optional __control1)
   (if __control1 (setq _control1 __control1)) _control1)
  (:control2
   (&optional __control2)
   (if __control2 (setq _control2 __control2)) _control2)
  (:control3
   (&optional __control3)
   (if __control3 (setq _control3 __control3)) _control3)
  (:control4
   (&optional __control4)
   (if __control4 (setq _control4 __control4)) _control4)
  (:serialization-length
   ()
   (+
    ;; time _time
    8
    ;; int32 _odom1
    4
    ;; int32 _odom2
    4
    ;; int32 _odom3
    4
    ;; int32 _odom4
    4
    ;; int32 _mode
    4
    ;; int32 _tank_id
    4
    ;; int32 _track_point_id
    4
    ;; int32 _first_alignment
    4
    ;; int32 _laser_alignment
    4
    ;; float32 _distance_alignment
    4
    ;; float32 _Pillar_distance
    4
    ;; int32 _pause
    4
    ;; int32 _stop
    4
    ;; int32 _back_home
    4
    ;; float32 _other_car_x
    4
    ;; float32 _other_car_y
    4
    ;; float32 _other_car_theta
    4
    ;; float32 _infrared_right
    4
    ;; float32 _infrared_left
    4
    ;; int32 _is_start_camera
    4
    ;; int32 _next_target_num
    4
    ;; int32 _control1
    4
    ;; int32 _control2
    4
    ;; int32 _control3
    4
    ;; int32 _control4
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _time
       (write-long (send _time :sec) s) (write-long (send _time :nsec) s)
     ;; int32 _odom1
       (write-long _odom1 s)
     ;; int32 _odom2
       (write-long _odom2 s)
     ;; int32 _odom3
       (write-long _odom3 s)
     ;; int32 _odom4
       (write-long _odom4 s)
     ;; int32 _mode
       (write-long _mode s)
     ;; int32 _tank_id
       (write-long _tank_id s)
     ;; int32 _track_point_id
       (write-long _track_point_id s)
     ;; int32 _first_alignment
       (write-long _first_alignment s)
     ;; int32 _laser_alignment
       (write-long _laser_alignment s)
     ;; float32 _distance_alignment
       (sys::poke _distance_alignment (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Pillar_distance
       (sys::poke _Pillar_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _pause
       (write-long _pause s)
     ;; int32 _stop
       (write-long _stop s)
     ;; int32 _back_home
       (write-long _back_home s)
     ;; float32 _other_car_x
       (sys::poke _other_car_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _other_car_y
       (sys::poke _other_car_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _other_car_theta
       (sys::poke _other_car_theta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _infrared_right
       (sys::poke _infrared_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _infrared_left
       (sys::poke _infrared_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _is_start_camera
       (write-long _is_start_camera s)
     ;; int32 _next_target_num
       (write-long _next_target_num s)
     ;; int32 _control1
       (write-long _control1 s)
     ;; int32 _control2
       (write-long _control2 s)
     ;; int32 _control3
       (write-long _control3 s)
     ;; int32 _control4
       (write-long _control4 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _time
     (send _time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _odom1
     (setq _odom1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _odom2
     (setq _odom2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _odom3
     (setq _odom3 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _odom4
     (setq _odom4 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _tank_id
     (setq _tank_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _track_point_id
     (setq _track_point_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _first_alignment
     (setq _first_alignment (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _laser_alignment
     (setq _laser_alignment (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _distance_alignment
     (setq _distance_alignment (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Pillar_distance
     (setq _Pillar_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _pause
     (setq _pause (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _stop
     (setq _stop (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _back_home
     (setq _back_home (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _other_car_x
     (setq _other_car_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _other_car_y
     (setq _other_car_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _other_car_theta
     (setq _other_car_theta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _infrared_right
     (setq _infrared_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _infrared_left
     (setq _infrared_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _is_start_camera
     (setq _is_start_camera (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _next_target_num
     (setq _next_target_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _control1
     (setq _control1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _control2
     (setq _control2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _control3
     (setq _control3 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _control4
     (setq _control4 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tcp2ros::readDataAll :md5sum-) "965a724e29296364637aadf1b2f664ed")
(setf (get tcp2ros::readDataAll :datatype-) "tcp2ros/readDataAll")
(setf (get tcp2ros::readDataAll :definition-)
      "time time
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

")



(provide :tcp2ros/readDataAll "965a724e29296364637aadf1b2f664ed")


