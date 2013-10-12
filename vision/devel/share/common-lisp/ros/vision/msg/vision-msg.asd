
(cl:in-package :asdf)

(defsystem "vision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RealObject" :depends-on ("_package_RealObject"))
    (:file "_package_RealObject" :depends-on ("_package"))
  ))