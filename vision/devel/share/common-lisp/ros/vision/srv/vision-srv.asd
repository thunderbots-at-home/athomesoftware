
(cl:in-package :asdf)

(defsystem "vision-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FindObject" :depends-on ("_package_FindObject"))
    (:file "_package_FindObject" :depends-on ("_package"))
    (:file "Match" :depends-on ("_package_Match"))
    (:file "_package_Match" :depends-on ("_package"))
    (:file "GetObjectsInScene" :depends-on ("_package_GetObjectsInScene"))
    (:file "_package_GetObjectsInScene" :depends-on ("_package"))
    (:file "Contains" :depends-on ("_package_Contains"))
    (:file "_package_Contains" :depends-on ("_package"))
  ))