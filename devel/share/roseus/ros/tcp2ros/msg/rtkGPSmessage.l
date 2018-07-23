;; Auto-generated. Do not edit!


(when (boundp 'tcp2ros::rtkGPSmessage)
  (if (not (find-package "TCP2ROS"))
    (make-package "TCP2ROS"))
  (shadow 'rtkGPSmessage (find-package "TCP2ROS")))
(unless (find-package "TCP2ROS::RTKGPSMESSAGE")
  (make-package "TCP2ROS::RTKGPSMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude rtkGPSmessage.msg.html


(defclass tcp2ros::rtkGPSmessage
  :super ros::object
  :slots (_ROS_time _GPS_time _vaild_flag _flash_state _north_meter _east_meter _yaw_rad ))

(defmethod tcp2ros::rtkGPSmessage
  (:init
   (&key
    ((:ROS_time __ROS_time) (instance ros::time :init))
    ((:GPS_time __GPS_time) "")
    ((:vaild_flag __vaild_flag) nil)
    ((:flash_state __flash_state) "")
    ((:north_meter __north_meter) 0.0)
    ((:east_meter __east_meter) 0.0)
    ((:yaw_rad __yaw_rad) 0.0)
    )
   (send-super :init)
   (setq _ROS_time __ROS_time)
   (setq _GPS_time (string __GPS_time))
   (setq _vaild_flag __vaild_flag)
   (setq _flash_state (string __flash_state))
   (setq _north_meter (float __north_meter))
   (setq _east_meter (float __east_meter))
   (setq _yaw_rad (float __yaw_rad))
   self)
  (:ROS_time
   (&optional __ROS_time)
   (if __ROS_time (setq _ROS_time __ROS_time)) _ROS_time)
  (:GPS_time
   (&optional __GPS_time)
   (if __GPS_time (setq _GPS_time __GPS_time)) _GPS_time)
  (:vaild_flag
   (&optional __vaild_flag)
   (if __vaild_flag (setq _vaild_flag __vaild_flag)) _vaild_flag)
  (:flash_state
   (&optional __flash_state)
   (if __flash_state (setq _flash_state __flash_state)) _flash_state)
  (:north_meter
   (&optional __north_meter)
   (if __north_meter (setq _north_meter __north_meter)) _north_meter)
  (:east_meter
   (&optional __east_meter)
   (if __east_meter (setq _east_meter __east_meter)) _east_meter)
  (:yaw_rad
   (&optional __yaw_rad)
   (if __yaw_rad (setq _yaw_rad __yaw_rad)) _yaw_rad)
  (:serialization-length
   ()
   (+
    ;; time _ROS_time
    8
    ;; string _GPS_time
    4 (length _GPS_time)
    ;; bool _vaild_flag
    1
    ;; string _flash_state
    4 (length _flash_state)
    ;; float64 _north_meter
    8
    ;; float64 _east_meter
    8
    ;; float64 _yaw_rad
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _ROS_time
       (write-long (send _ROS_time :sec) s) (write-long (send _ROS_time :nsec) s)
     ;; string _GPS_time
       (write-long (length _GPS_time) s) (princ _GPS_time s)
     ;; bool _vaild_flag
       (if _vaild_flag (write-byte -1 s) (write-byte 0 s))
     ;; string _flash_state
       (write-long (length _flash_state) s) (princ _flash_state s)
     ;; float64 _north_meter
       (sys::poke _north_meter (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _east_meter
       (sys::poke _east_meter (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw_rad
       (sys::poke _yaw_rad (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _ROS_time
     (send _ROS_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _ROS_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _GPS_time
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _GPS_time (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _vaild_flag
     (setq _vaild_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _flash_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _flash_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _north_meter
     (setq _north_meter (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _east_meter
     (setq _east_meter (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw_rad
     (setq _yaw_rad (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get tcp2ros::rtkGPSmessage :md5sum-) "579fa07aed9107c31ed915330a747d64")
(setf (get tcp2ros::rtkGPSmessage :datatype-) "tcp2ros/rtkGPSmessage")
(setf (get tcp2ros::rtkGPSmessage :definition-)
      "time ROS_time
string GPS_time
bool vaild_flag
string flash_state
float64 north_meter
float64 east_meter
float64 yaw_rad


")



(provide :tcp2ros/rtkGPSmessage "579fa07aed9107c31ed915330a747d64")


