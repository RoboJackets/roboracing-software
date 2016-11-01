;; Auto-generated. Do not edit!


(when (boundp 'avc::calibrate_image)
  (if (not (find-package "AVC"))
    (make-package "AVC"))
  (shadow 'calibrate_image (find-package "AVC")))
(unless (find-package "AVC::CALIBRATE_IMAGE")
  (make-package "AVC::CALIBRATE_IMAGE"))
(unless (find-package "AVC::CALIBRATE_IMAGEREQUEST")
  (make-package "AVC::CALIBRATE_IMAGEREQUEST"))
(unless (find-package "AVC::CALIBRATE_IMAGERESPONSE")
  (make-package "AVC::CALIBRATE_IMAGERESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))




(defclass avc::calibrate_imageRequest
  :super ros::object
  :slots (_image _imgDim _pixelsPerMeter _distanceToChessboard _chessboardDim _squareWidth ))

(defmethod avc::calibrate_imageRequest
  (:init
   (&key
    ((:image __image) (instance sensor_msgs::Image :init))
    ((:imgDim __imgDim) (make-array 2 :initial-element 0 :element-type :integer))
    ((:pixelsPerMeter __pixelsPerMeter) 0.0)
    ((:distanceToChessboard __distanceToChessboard) 0.0)
    ((:chessboardDim __chessboardDim) (make-array 2 :initial-element 0 :element-type :integer))
    ((:squareWidth __squareWidth) 0.0)
    )
   (send-super :init)
   (setq _image __image)
   (setq _imgDim __imgDim)
   (setq _pixelsPerMeter (float __pixelsPerMeter))
   (setq _distanceToChessboard (float __distanceToChessboard))
   (setq _chessboardDim __chessboardDim)
   (setq _squareWidth (float __squareWidth))
   self)
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:imgDim
   (&optional __imgDim)
   (if __imgDim (setq _imgDim __imgDim)) _imgDim)
  (:pixelsPerMeter
   (&optional __pixelsPerMeter)
   (if __pixelsPerMeter (setq _pixelsPerMeter __pixelsPerMeter)) _pixelsPerMeter)
  (:distanceToChessboard
   (&optional __distanceToChessboard)
   (if __distanceToChessboard (setq _distanceToChessboard __distanceToChessboard)) _distanceToChessboard)
  (:chessboardDim
   (&optional __chessboardDim)
   (if __chessboardDim (setq _chessboardDim __chessboardDim)) _chessboardDim)
  (:squareWidth
   (&optional __squareWidth)
   (if __squareWidth (setq _squareWidth __squareWidth)) _squareWidth)
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
    ;; int64[2] _imgDim
    (* 8    2)
    ;; float64 _pixelsPerMeter
    8
    ;; float64 _distanceToChessboard
    8
    ;; int64[2] _chessboardDim
    (* 8    2)
    ;; float64 _squareWidth
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
     ;; int64[2] _imgDim
     (dotimes (i 2)
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _imgDim i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _imgDim i)) (= (length ((elt _imgDim i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _imgDim i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _imgDim i) . bv) 1) -1) s))
             ((and (class (elt _imgDim i)) (= (length ((elt _imgDim i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _imgDim i) . bv) 0) s)
              (write-long (if (>= (elt _imgDim i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _imgDim i) s)(write-long (if (>= (elt _imgDim i) 0) 0 #xffffffff) s)))
       )
     ;; float64 _pixelsPerMeter
       (sys::poke _pixelsPerMeter (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _distanceToChessboard
       (sys::poke _distanceToChessboard (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64[2] _chessboardDim
     (dotimes (i 2)
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _chessboardDim i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _chessboardDim i)) (= (length ((elt _chessboardDim i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _chessboardDim i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _chessboardDim i) . bv) 1) -1) s))
             ((and (class (elt _chessboardDim i)) (= (length ((elt _chessboardDim i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _chessboardDim i) . bv) 0) s)
              (write-long (if (>= (elt _chessboardDim i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _chessboardDim i) s)(write-long (if (>= (elt _chessboardDim i) 0) 0 #xffffffff) s)))
       )
     ;; float64 _squareWidth
       (sys::poke _squareWidth (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;; int64[2] _imgDim
   (dotimes (i (length _imgDim))
#+(or :alpha :irix6 :x86_64)
      (setf (elt _imgDim i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _imgDim i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     )
   ;; float64 _pixelsPerMeter
     (setq _pixelsPerMeter (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _distanceToChessboard
     (setq _distanceToChessboard (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64[2] _chessboardDim
   (dotimes (i (length _chessboardDim))
#+(or :alpha :irix6 :x86_64)
      (setf (elt _chessboardDim i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _chessboardDim i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     )
   ;; float64 _squareWidth
     (setq _squareWidth (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass avc::calibrate_imageResponse
  :super ros::object
  :slots (_x ))

(defmethod avc::calibrate_imageResponse
  (:init
   (&key
    ((:x __x) nil)
    )
   (send-super :init)
   (setq _x __x)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:serialization-length
   ()
   (+
    ;; bool _x
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _x
       (if _x (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _x
     (setq _x (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass avc::calibrate_image
  :super ros::object
  :slots ())

(setf (get avc::calibrate_image :md5sum-) "40ee6ef1a25dadfb2e938d870cf568c0")
(setf (get avc::calibrate_image :datatype-) "avc/calibrate_image")
(setf (get avc::calibrate_image :request) avc::calibrate_imageRequest)
(setf (get avc::calibrate_image :response) avc::calibrate_imageResponse)

(defmethod avc::calibrate_imageRequest
  (:response () (instance avc::calibrate_imageResponse :init)))

(setf (get avc::calibrate_imageRequest :md5sum-) "40ee6ef1a25dadfb2e938d870cf568c0")
(setf (get avc::calibrate_imageRequest :datatype-) "avc/calibrate_imageRequest")
(setf (get avc::calibrate_imageRequest :definition-)
      "sensor_msgs/Image image
int64[2] imgDim
float64 pixelsPerMeter
float64 distanceToChessboard
int64[2] chessboardDim
float64 squareWidth

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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
---
bool x
")

(setf (get avc::calibrate_imageResponse :md5sum-) "40ee6ef1a25dadfb2e938d870cf568c0")
(setf (get avc::calibrate_imageResponse :datatype-) "avc/calibrate_imageResponse")
(setf (get avc::calibrate_imageResponse :definition-)
      "sensor_msgs/Image image
int64[2] imgDim
float64 pixelsPerMeter
float64 distanceToChessboard
int64[2] chessboardDim
float64 squareWidth

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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
---
bool x
")



(provide :avc/calibrate_image "40ee6ef1a25dadfb2e938d870cf568c0")


