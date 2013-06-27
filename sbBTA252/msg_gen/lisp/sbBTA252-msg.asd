
(cl:in-package :asdf)

(defsystem "sbBTA252-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SBBTA252Data" :depends-on ("_package_SBBTA252Data"))
    (:file "_package_SBBTA252Data" :depends-on ("_package"))
  ))