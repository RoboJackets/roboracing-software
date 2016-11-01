
(cl:in-package :asdf)

(defsystem "rr_platform-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "speed" :depends-on ("_package_speed"))
    (:file "_package_speed" :depends-on ("_package"))
    (:file "steering" :depends-on ("_package_steering"))
    (:file "_package_steering" :depends-on ("_package"))
  ))