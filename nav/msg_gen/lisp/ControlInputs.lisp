; Auto-generated. Do not edit!


(cl:in-package nav-msg)


;//! \htmlinclude ControlInputs.msg.html

(cl:defclass <ControlInputs> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (u_roll
    :reader u_roll
    :initarg :u_roll
    :type cl:float
    :initform 0.0)
   (u_pitch
    :reader u_pitch
    :initarg :u_pitch
    :type cl:float
    :initform 0.0)
   (u_yaw
    :reader u_yaw
    :initarg :u_yaw
    :type cl:float
    :initform 0.0)
   (u_depth
    :reader u_depth
    :initarg :u_depth
    :type cl:float
    :initform 0.0)
   (u_surge
    :reader u_surge
    :initarg :u_surge
    :type cl:float
    :initform 0.0))
)

(cl:defclass ControlInputs (<ControlInputs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlInputs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlInputs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nav-msg:<ControlInputs> is deprecated: use nav-msg:ControlInputs instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ControlInputs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nav-msg:header-val is deprecated.  Use nav-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'u_roll-val :lambda-list '(m))
(cl:defmethod u_roll-val ((m <ControlInputs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nav-msg:u_roll-val is deprecated.  Use nav-msg:u_roll instead.")
  (u_roll m))

(cl:ensure-generic-function 'u_pitch-val :lambda-list '(m))
(cl:defmethod u_pitch-val ((m <ControlInputs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nav-msg:u_pitch-val is deprecated.  Use nav-msg:u_pitch instead.")
  (u_pitch m))

(cl:ensure-generic-function 'u_yaw-val :lambda-list '(m))
(cl:defmethod u_yaw-val ((m <ControlInputs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nav-msg:u_yaw-val is deprecated.  Use nav-msg:u_yaw instead.")
  (u_yaw m))

(cl:ensure-generic-function 'u_depth-val :lambda-list '(m))
(cl:defmethod u_depth-val ((m <ControlInputs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nav-msg:u_depth-val is deprecated.  Use nav-msg:u_depth instead.")
  (u_depth m))

(cl:ensure-generic-function 'u_surge-val :lambda-list '(m))
(cl:defmethod u_surge-val ((m <ControlInputs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nav-msg:u_surge-val is deprecated.  Use nav-msg:u_surge instead.")
  (u_surge m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlInputs>) ostream)
  "Serializes a message object of type '<ControlInputs>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'u_roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'u_pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'u_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'u_depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'u_surge))))
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
    (cl:setf (cl:slot-value msg 'u_roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u_pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u_yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u_depth) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u_surge) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlInputs>)))
  "Returns string type for a message object of type '<ControlInputs>"
  "nav/ControlInputs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlInputs)))
  "Returns string type for a message object of type 'ControlInputs"
  "nav/ControlInputs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlInputs>)))
  "Returns md5sum for a message object of type '<ControlInputs>"
  "3b57c87c1174acc91a3d8f917b790174")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlInputs)))
  "Returns md5sum for a message object of type 'ControlInputs"
  "3b57c87c1174acc91a3d8f917b790174")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlInputs>)))
  "Returns full string definition for message of type '<ControlInputs>"
  (cl:format cl:nil "Header header~%float32 u_roll~%float32 u_pitch~%float32 u_yaw~%float32 u_depth~%float32 u_surge~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlInputs)))
  "Returns full string definition for message of type 'ControlInputs"
  (cl:format cl:nil "Header header~%float32 u_roll~%float32 u_pitch~%float32 u_yaw~%float32 u_depth~%float32 u_surge~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
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
    (cl:cons ':u_roll (u_roll msg))
    (cl:cons ':u_pitch (u_pitch msg))
    (cl:cons ':u_yaw (u_yaw msg))
    (cl:cons ':u_depth (u_depth msg))
    (cl:cons ':u_surge (u_surge msg))
))
