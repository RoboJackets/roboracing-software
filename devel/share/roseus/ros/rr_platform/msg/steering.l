;; Auto-generated. Do not edit!


(when (boundp 'rr_platform::steering)
  (if (not (find-package "RR_PLATFORM"))
    (make-package "RR_PLATFORM"))
  (shadow 'steering (find-package "RR_PLATFORM")))
(unless (find-package "RR_PLATFORM::STEERING")
  (make-package "RR_PLATFORM::STEERING"))

(in-package "ROS")
;;//! \htmlinclude steering.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rr_platform::steering
  :super ros::object
  :slots (_header _angle ))

(defmethod rr_platform::steering
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _angle (float __angle))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rr_platform::steering :md5sum-) "84c1d14f72a90efbf3b1a4de632c5bfb")
(setf (get rr_platform::steering :datatype-) "rr_platform/steering")
(setf (get rr_platform::steering :definition-)
      "# Message for controlling the RC car servo and motors
Header header           # timestamp in the header is the time the command is sent
float64 angle             # front wheel angle

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



(provide :rr_platform/steering "84c1d14f72a90efbf3b1a4de632c5bfb")


