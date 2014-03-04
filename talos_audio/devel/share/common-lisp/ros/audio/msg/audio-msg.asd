
(cl:in-package :asdf)

(defsystem "audio-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FRClientGoal" :depends-on ("_package_FRClientGoal"))
    (:file "_package_FRClientGoal" :depends-on ("_package"))
  ))