
(cl:in-package :asdf)

(defsystem "nav-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ControlInputs" :depends-on ("_package_ControlInputs"))
    (:file "_package_ControlInputs" :depends-on ("_package"))
  ))