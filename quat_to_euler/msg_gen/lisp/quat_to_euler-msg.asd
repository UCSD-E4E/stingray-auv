
(cl:in-package :asdf)

(defsystem "quat_to_euler-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Eulers" :depends-on ("_package_Eulers"))
    (:file "_package_Eulers" :depends-on ("_package"))
  ))