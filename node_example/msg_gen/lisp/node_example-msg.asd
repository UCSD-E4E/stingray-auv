
(cl:in-package :asdf)

(defsystem "node_example-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "node_example_data" :depends-on ("_package_node_example_data"))
    (:file "_package_node_example_data" :depends-on ("_package"))
  ))