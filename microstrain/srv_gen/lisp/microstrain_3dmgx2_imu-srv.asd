
(cl:in-package :asdf)

(defsystem "microstrain_3dmgx2_imu-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AddOffset" :depends-on ("_package_AddOffset"))
    (:file "_package_AddOffset" :depends-on ("_package"))
  ))