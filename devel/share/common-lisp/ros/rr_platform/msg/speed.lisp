; Auto-generated. Do not edit!


(cl:in-package rr_platform-msg)


;//! \htmlinclude speed.msg.html

(cl:defclass <speed> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass speed (<speed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <speed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'speed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rr_platform-msg:<speed> is deprecated: use rr_platform-msg:speed instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <speed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rr_platform-msg:header-val is deprecated.  Use rr_platform-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <speed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rr_platform-msg:speed-val is deprecated.  Use rr_platform-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <speed>) ostream)
  "Serializes a message object of type '<speed>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <speed>) istream)
  "Deserializes a message object of type '<speed>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<speed>)))
  "Returns string type for a message object of type '<speed>"
  "rr_platform/speed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'speed)))
  "Returns string type for a message object of type 'speed"
  "rr_platform/speed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<speed>)))
  "Returns md5sum for a message object of type '<speed>"
  "17b1dfb61067b5181d2fbfcebd547b10")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'speed)))
  "Returns md5sum for a message object of type 'speed"
  "17b1dfb61067b5181d2fbfcebd547b10")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<speed>)))
  "Returns full string definition for message of type '<speed>"
  (cl:format cl:nil "# Message for controlling the RC car servo and motors~%Header header           # timestamp in the header is the time the command is sent~%float64 speed             # wheel speed~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'speed)))
  "Returns full string definition for message of type 'speed"
  (cl:format cl:nil "# Message for controlling the RC car servo and motors~%Header header           # timestamp in the header is the time the command is sent~%float64 speed             # wheel speed~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <speed>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <speed>))
  "Converts a ROS message object to a list"
  (cl:list 'speed
    (cl:cons ':header (header msg))
    (cl:cons ':speed (speed msg))
))
