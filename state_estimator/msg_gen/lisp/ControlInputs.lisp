; Auto-generated. Do not edit!


(cl:in-package state_estimator-msg)


;//! \htmlinclude ControlInputs.msg.html

(cl:defclass <ControlInputs> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (uRoll
    :reader uRoll
    :initarg :uRoll
    :type cl:float
    :initform 0.0)
   (uPitch
    :reader uPitch
    :initarg :uPitch
    :type cl:float
    :initform 0.0)
   (uYaw
    :reader uYaw
    :initarg :uYaw
    :type cl:float
    :initform 0.0)
   (uDepth
    :reader uDepth
    :initarg :uDepth
    :type cl:float
    :initform 0.0)
   (uSurge
    :reader uSurge
    :initarg :uSurge
    :type cl:float
    :initform 0.0))
)

(cl:defclass ControlInputs (<ControlInputs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlInputs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlInputs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator-msg:<ControlInputs> is deprecated: use state_estimator-msg:ControlInputs instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ControlInputs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator-msg:header-val is deprecated.  Use state_estimator-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'uRoll-val :lambda-list '(m))
(cl:defmethod uRoll-val ((m <ControlInputs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator-msg:uRoll-val is deprecated.  Use state_estimator-msg:uRoll instead.")
  (uRoll m))

(cl:ensure-generic-function 'uPitch-val :lambda-list '(m))
(cl:defmethod uPitch-val ((m <ControlInputs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator-msg:uPitch-val is deprecated.  Use state_estimator-msg:uPitch instead.")
  (uPitch m))

(cl:ensure-generic-function 'uYaw-val :lambda-list '(m))
(cl:defmethod uYaw-val ((m <ControlInputs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator-msg:uYaw-val is deprecated.  Use state_estimator-msg:uYaw instead.")
  (uYaw m))

(cl:ensure-generic-function 'uDepth-val :lambda-list '(m))
(cl:defmethod uDepth-val ((m <ControlInputs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator-msg:uDepth-val is deprecated.  Use state_estimator-msg:uDepth instead.")
  (uDepth m))

(cl:ensure-generic-function 'uSurge-val :lambda-list '(m))
(cl:defmethod uSurge-val ((m <ControlInputs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator-msg:uSurge-val is deprecated.  Use state_estimator-msg:uSurge instead.")
  (uSurge m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlInputs>) ostream)
  "Serializes a message object of type '<ControlInputs>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'uRoll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'uPitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'uYaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'uDepth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'uSurge))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlInputs>) istream)
  "Deserializes a message object of type '<ControlInputs>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'uRoll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'uPitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'uYaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'uDepth) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'uSurge) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlInputs>)))
  "Returns string type for a message object of type '<ControlInputs>"
  "state_estimator/ControlInputs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlInputs)))
  "Returns string type for a message object of type 'ControlInputs"
  "state_estimator/ControlInputs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlInputs>)))
  "Returns md5sum for a message object of type '<ControlInputs>"
  "7e85360b8387e61230e8b83a30130b68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlInputs)))
  "Returns md5sum for a message object of type 'ControlInputs"
  "7e85360b8387e61230e8b83a30130b68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlInputs>)))
  "Returns full string definition for message of type '<ControlInputs>"
  (cl:format cl:nil "Header header~%float32 uRoll~%float32 uPitch~%float32 uYaw~%float32 uDepth~%float32 uSurge~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlInputs)))
  "Returns full string definition for message of type 'ControlInputs"
  (cl:format cl:nil "Header header~%float32 uRoll~%float32 uPitch~%float32 uYaw~%float32 uDepth~%float32 uSurge~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlInputs>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlInputs>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlInputs
    (cl:cons ':header (header msg))
    (cl:cons ':uRoll (uRoll msg))
    (cl:cons ':uPitch (uPitch msg))
    (cl:cons ':uYaw (uYaw msg))
    (cl:cons ':uDepth (uDepth msg))
    (cl:cons ':uSurge (uSurge msg))
))
