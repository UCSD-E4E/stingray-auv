; Auto-generated. Do not edit!


(cl:in-package state_estimator-msg)


;//! \htmlinclude PrimePowerStartStop.msg.html

(cl:defclass <PrimePowerStartStop> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (primePowerStartStop
    :reader primePowerStartStop
    :initarg :primePowerStartStop
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PrimePowerStartStop (<PrimePowerStartStop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PrimePowerStartStop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PrimePowerStartStop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator-msg:<PrimePowerStartStop> is deprecated: use state_estimator-msg:PrimePowerStartStop instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PrimePowerStartStop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator-msg:header-val is deprecated.  Use state_estimator-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'primePowerStartStop-val :lambda-list '(m))
(cl:defmethod primePowerStartStop-val ((m <PrimePowerStartStop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator-msg:primePowerStartStop-val is deprecated.  Use state_estimator-msg:primePowerStartStop instead.")
  (primePowerStartStop m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PrimePowerStartStop>) ostream)
  "Serializes a message object of type '<PrimePowerStartStop>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'primePowerStartStop) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PrimePowerStartStop>) istream)
  "Deserializes a message object of type '<PrimePowerStartStop>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'primePowerStartStop) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PrimePowerStartStop>)))
  "Returns string type for a message object of type '<PrimePowerStartStop>"
  "state_estimator/PrimePowerStartStop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PrimePowerStartStop)))
  "Returns string type for a message object of type 'PrimePowerStartStop"
  "state_estimator/PrimePowerStartStop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PrimePowerStartStop>)))
  "Returns md5sum for a message object of type '<PrimePowerStartStop>"
  "8c1703c7a390022e30f74bcfbb99f9be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PrimePowerStartStop)))
  "Returns md5sum for a message object of type 'PrimePowerStartStop"
  "8c1703c7a390022e30f74bcfbb99f9be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PrimePowerStartStop>)))
  "Returns full string definition for message of type '<PrimePowerStartStop>"
  (cl:format cl:nil "Header header~%bool primePowerStartStop~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PrimePowerStartStop)))
  "Returns full string definition for message of type 'PrimePowerStartStop"
  (cl:format cl:nil "Header header~%bool primePowerStartStop~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PrimePowerStartStop>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PrimePowerStartStop>))
  "Converts a ROS message object to a list"
  (cl:list 'PrimePowerStartStop
    (cl:cons ':header (header msg))
    (cl:cons ':primePowerStartStop (primePowerStartStop msg))
))
