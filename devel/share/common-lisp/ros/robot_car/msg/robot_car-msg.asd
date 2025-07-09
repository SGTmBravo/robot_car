
(cl:in-package :asdf)

(defsystem "robot_car-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "WheelVelocities" :depends-on ("_package_WheelVelocities"))
    (:file "_package_WheelVelocities" :depends-on ("_package"))
  ))