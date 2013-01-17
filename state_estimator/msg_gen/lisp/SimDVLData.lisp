; Auto-generated. Do not edit!


(cl:in-package state_estimator-msg)


;//! \htmlinclude SimDVLData.msg.html

(cl:defclass <SimDVLData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (surge
    :reader surge
    :initarg :surge
    :type cl:float
    :initform 0.0)
   (sway
    :reader sway
    :initarg :sway
    :type cl:float
    :initform 0.0)
   (heave
    :reader heave
    :initarg :heave
    :type cl:float
    :initform 0.0)
   (altitude
    :reader altitude
    :initarg :altitude
    :type cl:float
    :initform 0.0))
)

(cl:defclass SimDVLData (<SimDVLData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SimDVLData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SimDVLData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator-msg:<SimDVLData> is deprecated: use state_estimator-msg:SimDVLData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SimDVLData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator-msg:header-val is deprecated.  Use state_estimator-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'surge-val :lambda-list '(m))
(cl:defmethod surge-val ((m <SimDVLData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator-msg:surge-val is deprecated.  Use state_estimator-msg:surge instead.")
  (surge m))

(cl:ensure-generic-function 'sway-val :lambda-list '(m))
(cl:defmethod sway-val ((m <SimDVLData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator-msg:sway-val is deprecated.  Use state_estimator-msg:sway instead.")
  (sway m))

(cl:ensure-generic-function 'heave-val :lambda-list '(m))
(cl:defmethod heave-val ((m <SimDVLData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator-msg:heave-val is deprecated.  Use state_estimator-msg:heave instead.")
  (heave m))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <SimDVLData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator-msg:altitude-val is deprecated.  Use state_estimator-msg:altitude instead.")
  (altitude m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SimDVLData>) ostream)
  "Serializes a message object of type '<SimDVLData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'surge))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sway))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'heave))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'altitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SimDVLData>) istream)
  "Deserializes a message object of type '<SimDVLData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'surge) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sway) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heave) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altitude) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SimDVLData>)))
  "Returns string type for a message object of type '<SimDVLData>"
  "state_estimator/SimDVLData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimDVLData)))
  "Returns string type for a message object of type 'SimDVLData"
  "state_estimator/SimDVLData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SimDVLData>)))
  "Returns md5sum for a message object of type '<SimDVLData>"
  "7a37cc100a2e4608bcf0bcadc3a7a430")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SimDVLData)))
  "Returns md5sum for a message object of type 'SimDVLData"
  "7a37cc100a2e4608bcf0bcadc3a7a430")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SimDVLData>)))
  "Returns full string definition for message of type '<SimDVLData>"
  (cl:format cl:nil "Header header~%float32 surge~%float32 sway~%float32 heave~%float32 altitude~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SimDVLData)))
  "Returns full string definition for message of type 'SimDVLData"
  (cl:format cl:nil "Header header~%float32 surge~%float32 sway~%float32 heave~%float32 altitude~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SimDVLData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SimDVLData>))
  "Converts a ROS message object to a list"
  (cl:list 'SimDVLData
    (cl:cons ':header (header msg))
    (cl:cons ':surge (surge msg))
    (cl:cons ':sway (sway msg))
    (cl:cons ':heave (heave msg))
    (cl:cons ':altitude (altitude msg))
))
