
(cl:in-package :asdf)

(defsystem "visual_servo_test-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CartesianPose" :depends-on ("_package_CartesianPose"))
    (:file "_package_CartesianPose" :depends-on ("_package"))
  ))