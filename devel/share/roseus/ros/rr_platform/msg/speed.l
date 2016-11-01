;; Auto-generated. Do not edit!


(when (boundp 'rr_platform::speed)
  (if (not (find-package "RR_PLATFORM"))
    (make-package "RR_PLATFORM"))
  (shadow 'speed (find-package "RR_PLATFORM")))
(unless (find-package "RR_PLATFORM::SPEED")
  (make-package "RR_PLATFORM::SPEED"))

(in-package "ROS")
;;//! \htmlinclude speed.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rr_platform::speed
  :super ros::object
  :slots (_header _speed ))

(defmethod rr_platform::speed
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _speed (float __speed))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _speed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _speed
     (setq _speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rr_platform::speed :md5sum-) "17b1dfb61067b5181d2fbfcebd547b10")
(setf (get rr_platform::speed :datatype-) "rr_platform/speed")
(setf (get rr_platform::speed :definition-)
      "# Message for controlling the RC car servo and motors
Header header           # timestamp in the header is the time the command is sent
float64 speed             # wheel speed

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :rr_platform/speed "17b1dfb61067b5181d2fbfcebd547b10")


