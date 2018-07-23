;; Auto-generated. Do not edit!


(when (boundp 'tcp2ros::reach)
  (if (not (find-package "TCP2ROS"))
    (make-package "TCP2ROS"))
  (shadow 'reach (find-package "TCP2ROS")))
(unless (find-package "TCP2ROS::REACH")
  (make-package "TCP2ROS::REACH"))

(in-package "ROS")
;;//! \htmlinclude reach.msg.html


(defclass tcp2ros::reach
  :super ros::object
  :slots (_time _reach ))

(defmethod tcp2ros::reach
  (:init
   (&key
    ((:time __time) (instance ros::time :init))
    ((:reach __reach) 0)
    )
   (send-super :init)
   (setq _time __time)
   (setq _reach (round __reach))
   self)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:reach
   (&optional __reach)
   (if __reach (setq _reach __reach)) _reach)
  (:serialization-length
   ()
   (+
    ;; time _time
    8
    ;; int32 _reach
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _time
       (write-long (send _time :sec) s) (write-long (send _time :nsec) s)
     ;; int32 _reach
       (write-long _reach s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _time
     (send _time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _reach
     (setq _reach (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tcp2ros::reach :md5sum-) "2dcc348a5da28ada0f2c0d073cc9de9a")
(setf (get tcp2ros::reach :datatype-) "tcp2ros/reach")
(setf (get tcp2ros::reach :definition-)
      "time time
int32 reach

")



(provide :tcp2ros/reach "2dcc348a5da28ada0f2c0d073cc9de9a")


