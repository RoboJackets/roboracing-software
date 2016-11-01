; Auto-generated. Do not edit!


(cl:in-package avc-srv)


;//! \htmlinclude transform_image-request.msg.html

(cl:defclass <transform_image-request> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass transform_image-request (<transform_image-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <transform_image-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'transform_image-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avc-srv:<transform_image-request> is deprecated: use avc-srv:transform_image-request instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <transform_image-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avc-srv:image-val is deprecated.  Use avc-srv:image instead.")
  (image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <transform_image-request>) ostream)
  "Serializes a message object of type '<transform_image-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <transform_image-request>) istream)
  "Deserializes a message object of type '<transform_image-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<transform_image-request>)))
  "Returns string type for a service object of type '<transform_image-request>"
  "avc/transform_imageRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'transform_image-request)))
  "Returns string type for a service object of type 'transform_image-request"
  "avc/transform_imageRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<transform_image-request>)))
  "Returns md5sum for a message object of type '<transform_image-request>"
  "e2a28c9f46ac03baf18b533601a3ca80")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'transform_image-request)))
  "Returns md5sum for a message object of type 'transform_image-request"
  "e2a28c9f46ac03baf18b533601a3ca80")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<transform_image-request>)))
  "Returns full string definition for message of type '<transform_image-request>"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'transform_image-request)))
  "Returns full string definition for message of type 'transform_image-request"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <transform_image-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <transform_image-request>))
  "Converts a ROS message object to a list"
  (cl:list 'transform_image-request
    (cl:cons ':image (image msg))
))
;//! \htmlinclude transform_image-response.msg.html

(cl:defclass <transform_image-response> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass transform_image-response (<transform_image-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <transform_image-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'transform_image-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avc-srv:<transform_image-response> is deprecated: use avc-srv:transform_image-response instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <transform_image-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avc-srv:image-val is deprecated.  Use avc-srv:image instead.")
  (image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <transform_image-response>) ostream)
  "Serializes a message object of type '<transform_image-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <transform_image-response>) istream)
  "Deserializes a message object of type '<transform_image-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<transform_image-response>)))
  "Returns string type for a service object of type '<transform_image-response>"
  "avc/transform_imageResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'transform_image-response)))
  "Returns string type for a service object of type 'transform_image-response"
  "avc/transform_imageResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<transform_image-response>)))
  "Returns md5sum for a message object of type '<transform_image-response>"
  "e2a28c9f46ac03baf18b533601a3ca80")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'transform_image-response)))
  "Returns md5sum for a message object of type 'transform_image-response"
  "e2a28c9f46ac03baf18b533601a3ca80")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<transform_image-response>)))
  "Returns full string definition for message of type '<transform_image-response>"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'transform_image-response)))
  "Returns full string definition for message of type 'transform_image-response"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <transform_image-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <transform_image-response>))
  "Converts a ROS message object to a list"
  (cl:list 'transform_image-response
    (cl:cons ':image (image msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'transform_image)))
  'transform_image-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'transform_image)))
  'transform_image-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'transform_image)))
  "Returns string type for a service object of type '<transform_image>"
  "avc/transform_image")