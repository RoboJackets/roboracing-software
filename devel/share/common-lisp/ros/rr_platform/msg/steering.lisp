; Auto-generated. Do not edit!


(cl:in-package rr_platform-msg)


;//! \htmlinclude steering.msg.html

(cl:defclass <steering> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass steering (<steering>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <steering>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'steering)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rr_platform-msg:<steering> is deprecated: use rr_platform-msg:steering instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <steering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rr_platform-msg:header-val is deprecated.  Use rr_platform-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <steering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rr_platform-msg:angle-val is deprecated.  Use rr_platform-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <steering>) ostream)
  "Serializes a message object of type '<steering>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <steering>) istream)
  "Deserializes a message object of type '<steering>"
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
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<steering>)))
  "Returns string type for a message object of type '<steering>"
  "rr_platform/steering")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'steering)))
  "Returns string type for a message object of type 'steering"
  "rr_platform/steering")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<steering>)))
  "Returns md5sum for a message object of type '<steering>"
  "84c1d14f72a90efbf3b1a4de632c5bfb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'steering)))
  "Returns md5sum for a message object of type 'steering"
  "84c1d14f72a90efbf3b1a4de632c5bfb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<steering>)))
  "Returns full string definition for message of type '<steering>"
  (cl:format cl:nil "# Message for controlling the RC car servo and motors~%Header header           # timestamp in the header is the time the command is sent~%float64 angle             # front wheel angle~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'steering)))
  "Returns full string definition for message of type 'steering"
  (cl:format cl:nil "# Message for controlling the RC car servo and motors~%Header header           # timestamp in the header is the time the command is sent~%float64 angle             # front wheel angle~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <steering>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <steering>))
  "Converts a ROS message object to a list"
  (cl:list 'steering
    (cl:cons ':header (header msg))
    (cl:cons ':angle (angle msg))
))
