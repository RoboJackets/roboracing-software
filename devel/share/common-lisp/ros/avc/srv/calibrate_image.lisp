; Auto-generated. Do not edit!


(cl:in-package avc-srv)


;//! \htmlinclude calibrate_image-request.msg.html

(cl:defclass <calibrate_image-request> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (imgDim
    :reader imgDim
    :initarg :imgDim
    :type (cl:vector cl:integer)
   :initform (cl:make-array 2 :element-type 'cl:integer :initial-element 0))
   (pixelsPerMeter
    :reader pixelsPerMeter
    :initarg :pixelsPerMeter
    :type cl:float
    :initform 0.0)
   (distanceToChessboard
    :reader distanceToChessboard
    :initarg :distanceToChessboard
    :type cl:float
    :initform 0.0)
   (chessboardDim
    :reader chessboardDim
    :initarg :chessboardDim
    :type (cl:vector cl:integer)
   :initform (cl:make-array 2 :element-type 'cl:integer :initial-element 0))
   (squareWidth
    :reader squareWidth
    :initarg :squareWidth
    :type cl:float
    :initform 0.0))
)

(cl:defclass calibrate_image-request (<calibrate_image-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <calibrate_image-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'calibrate_image-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avc-srv:<calibrate_image-request> is deprecated: use avc-srv:calibrate_image-request instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <calibrate_image-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avc-srv:image-val is deprecated.  Use avc-srv:image instead.")
  (image m))

(cl:ensure-generic-function 'imgDim-val :lambda-list '(m))
(cl:defmethod imgDim-val ((m <calibrate_image-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avc-srv:imgDim-val is deprecated.  Use avc-srv:imgDim instead.")
  (imgDim m))

(cl:ensure-generic-function 'pixelsPerMeter-val :lambda-list '(m))
(cl:defmethod pixelsPerMeter-val ((m <calibrate_image-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avc-srv:pixelsPerMeter-val is deprecated.  Use avc-srv:pixelsPerMeter instead.")
  (pixelsPerMeter m))

(cl:ensure-generic-function 'distanceToChessboard-val :lambda-list '(m))
(cl:defmethod distanceToChessboard-val ((m <calibrate_image-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avc-srv:distanceToChessboard-val is deprecated.  Use avc-srv:distanceToChessboard instead.")
  (distanceToChessboard m))

(cl:ensure-generic-function 'chessboardDim-val :lambda-list '(m))
(cl:defmethod chessboardDim-val ((m <calibrate_image-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avc-srv:chessboardDim-val is deprecated.  Use avc-srv:chessboardDim instead.")
  (chessboardDim m))

(cl:ensure-generic-function 'squareWidth-val :lambda-list '(m))
(cl:defmethod squareWidth-val ((m <calibrate_image-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avc-srv:squareWidth-val is deprecated.  Use avc-srv:squareWidth instead.")
  (squareWidth m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <calibrate_image-request>) ostream)
  "Serializes a message object of type '<calibrate_image-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    ))
   (cl:slot-value msg 'imgDim))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pixelsPerMeter))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distanceToChessboard))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    ))
   (cl:slot-value msg 'chessboardDim))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'squareWidth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <calibrate_image-request>) istream)
  "Deserializes a message object of type '<calibrate_image-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  (cl:setf (cl:slot-value msg 'imgDim) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'imgDim)))
    (cl:dotimes (i 2)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pixelsPerMeter) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distanceToChessboard) (roslisp-utils:decode-double-float-bits bits)))
  (cl:setf (cl:slot-value msg 'chessboardDim) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'chessboardDim)))
    (cl:dotimes (i 2)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'squareWidth) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<calibrate_image-request>)))
  "Returns string type for a service object of type '<calibrate_image-request>"
  "avc/calibrate_imageRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'calibrate_image-request)))
  "Returns string type for a service object of type 'calibrate_image-request"
  "avc/calibrate_imageRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<calibrate_image-request>)))
  "Returns md5sum for a message object of type '<calibrate_image-request>"
  "40ee6ef1a25dadfb2e938d870cf568c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'calibrate_image-request)))
  "Returns md5sum for a message object of type 'calibrate_image-request"
  "40ee6ef1a25dadfb2e938d870cf568c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<calibrate_image-request>)))
  "Returns full string definition for message of type '<calibrate_image-request>"
  (cl:format cl:nil "sensor_msgs/Image image~%int64[2] imgDim~%float64 pixelsPerMeter~%float64 distanceToChessboard~%int64[2] chessboardDim~%float64 squareWidth~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'calibrate_image-request)))
  "Returns full string definition for message of type 'calibrate_image-request"
  (cl:format cl:nil "sensor_msgs/Image image~%int64[2] imgDim~%float64 pixelsPerMeter~%float64 distanceToChessboard~%int64[2] chessboardDim~%float64 squareWidth~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <calibrate_image-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'imgDim) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     8
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'chessboardDim) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <calibrate_image-request>))
  "Converts a ROS message object to a list"
  (cl:list 'calibrate_image-request
    (cl:cons ':image (image msg))
    (cl:cons ':imgDim (imgDim msg))
    (cl:cons ':pixelsPerMeter (pixelsPerMeter msg))
    (cl:cons ':distanceToChessboard (distanceToChessboard msg))
    (cl:cons ':chessboardDim (chessboardDim msg))
    (cl:cons ':squareWidth (squareWidth msg))
))
;//! \htmlinclude calibrate_image-response.msg.html

(cl:defclass <calibrate_image-response> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass calibrate_image-response (<calibrate_image-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <calibrate_image-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'calibrate_image-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avc-srv:<calibrate_image-response> is deprecated: use avc-srv:calibrate_image-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <calibrate_image-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avc-srv:x-val is deprecated.  Use avc-srv:x instead.")
  (x m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <calibrate_image-response>) ostream)
  "Serializes a message object of type '<calibrate_image-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'x) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <calibrate_image-response>) istream)
  "Deserializes a message object of type '<calibrate_image-response>"
    (cl:setf (cl:slot-value msg 'x) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<calibrate_image-response>)))
  "Returns string type for a service object of type '<calibrate_image-response>"
  "avc/calibrate_imageResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'calibrate_image-response)))
  "Returns string type for a service object of type 'calibrate_image-response"
  "avc/calibrate_imageResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<calibrate_image-response>)))
  "Returns md5sum for a message object of type '<calibrate_image-response>"
  "40ee6ef1a25dadfb2e938d870cf568c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'calibrate_image-response)))
  "Returns md5sum for a message object of type 'calibrate_image-response"
  "40ee6ef1a25dadfb2e938d870cf568c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<calibrate_image-response>)))
  "Returns full string definition for message of type '<calibrate_image-response>"
  (cl:format cl:nil "bool x~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'calibrate_image-response)))
  "Returns full string definition for message of type 'calibrate_image-response"
  (cl:format cl:nil "bool x~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <calibrate_image-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <calibrate_image-response>))
  "Converts a ROS message object to a list"
  (cl:list 'calibrate_image-response
    (cl:cons ':x (x msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'calibrate_image)))
  'calibrate_image-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'calibrate_image)))
  'calibrate_image-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'calibrate_image)))
  "Returns string type for a service object of type '<calibrate_image>"
  "avc/calibrate_image")