
(cl:in-package :asdf)

(defsystem "cse_kinect-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PoseData" :depends-on ("_package_PoseData"))
    (:file "_package_PoseData" :depends-on ("_package"))
  ))