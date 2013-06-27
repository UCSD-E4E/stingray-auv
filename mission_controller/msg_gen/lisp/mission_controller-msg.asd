
(cl:in-package :asdf)

(defsystem "mission_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "TargetWaypoints" :depends-on ("_package_TargetWaypoints"))
    (:file "_package_TargetWaypoints" :depends-on ("_package"))
  ))