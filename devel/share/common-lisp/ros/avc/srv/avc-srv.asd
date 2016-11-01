
(cl:in-package :asdf)

(defsystem "avc-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "calibrate_image" :depends-on ("_package_calibrate_image"))
    (:file "_package_calibrate_image" :depends-on ("_package"))
    (:file "transform_image" :depends-on ("_package_transform_image"))
    (:file "_package_transform_image" :depends-on ("_package"))
  ))