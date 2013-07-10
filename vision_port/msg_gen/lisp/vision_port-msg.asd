
(cl:in-package :asdf)

(defsystem "vision_port-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "visionGoal" :depends-on ("_package_visionGoal"))
    (:file "_package_visionGoal" :depends-on ("_package"))
    (:file "visionResult" :depends-on ("_package_visionResult"))
    (:file "_package_visionResult" :depends-on ("_package"))
    (:file "visionActionGoal" :depends-on ("_package_visionActionGoal"))
    (:file "_package_visionActionGoal" :depends-on ("_package"))
    (:file "visionFeedback" :depends-on ("_package_visionFeedback"))
    (:file "_package_visionFeedback" :depends-on ("_package"))
    (:file "visionActionResult" :depends-on ("_package_visionActionResult"))
    (:file "_package_visionActionResult" :depends-on ("_package"))
    (:file "visionActionFeedback" :depends-on ("_package_visionActionFeedback"))
    (:file "_package_visionActionFeedback" :depends-on ("_package"))
    (:file "visionAction" :depends-on ("_package_visionAction"))
    (:file "_package_visionAction" :depends-on ("_package"))
  ))