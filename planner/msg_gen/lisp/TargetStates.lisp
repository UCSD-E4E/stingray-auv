; Auto-generated. Do not edit!


(cl:in-package planner-msg)


;//! \htmlinclude TargetStates.msg.html

(cl:defclass <TargetStates> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (target_roll
    :reader target_roll
    :initarg :target_roll
    :type cl:float
    :initform 0.0)
   (target_pitch
    :reader target_pitch
    :initarg :target_pitch
    :type cl:float
    :initform 0.0)
   (target_yaw
    :reader target_yaw
    :initarg :target_yaw
    :type cl:float
    :initform 0.0)
   (target_depth
    :reader target_depth
    :initarg :target_depth
    :type cl:float
    :initform 0.0)
   (target_surge
    :reader target_surge
    :initarg :target_surge
    :type cl:float
    :initform 0.0)
   (target_lat
    :reader target_lat
    :initarg :target_lat
    :type cl:float
    :initform 0.0)
   (target_lon
    :reader target_lon
    :initarg :target_lon
    :type cl:float
    :initform 0.0))
)

(cl:defclass TargetStates (<TargetStates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetStates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetStates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planner-msg:<TargetStates> is deprecated: use planner-msg:TargetStates instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TargetStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner-msg:header-val is deprecated.  Use planner-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'target_roll-val :lambda-list '(m))
(cl:defmethod target_roll-val ((m <TargetStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner-msg:target_roll-val is deprecated.  Use planner-msg:target_roll instead.")
  (target_roll m))

(cl:ensure-generic-function 'target_pitch-val :lambda-list '(m))
(cl:defmethod target_pitch-val ((m <TargetStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner-msg:target_pitch-val is deprecated.  Use planner-msg:target_pitch instead.")
  (target_pitch m))

(cl:ensure-generic-function 'target_yaw-val :lambda-list '(m))
(cl:defmethod target_yaw-val ((m <TargetStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner-msg:target_yaw-val is deprecated.  Use planner-msg:target_yaw instead.")
  (target_yaw m))

(cl:ensure-generic-function 'target_depth-val :lambda-list '(m))
(cl:defmethod target_depth-val ((m <TargetStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner-msg:target_depth-val is deprecated.  Use planner-msg:target_depth instead.")
  (target_depth m))

(cl:ensure-generic-function 'target_surge-val :lambda-list '(m))
(cl:defmethod target_surge-val ((m <TargetStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner-msg:target_surge-val is deprecated.  Use planner-msg:target_surge instead.")
  (target_surge m))

(cl:ensure-generic-function 'target_lat-val :lambda-list '(m))
(cl:defmethod target_lat-val ((m <TargetStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner-msg:target_lat-val is deprecated.  Use planner-msg:target_lat instead.")
  (target_lat m))

(cl:ensure-generic-function 'target_lon-val :lambda-list '(m))
(cl:defmethod target_lon-val ((m <TargetStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner-msg:target_lon-val is deprecated.  Use planner-msg:target_lon instead.")
  (target_lon m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetStates>) ostream)
  "Serializes a message object of type '<TargetStates>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_surge))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_lat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_lon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetStates>) istream)
  "Deserializes a message object of type '<TargetStates>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_depth) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_surge) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_lat) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_lon) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetStates>)))
  "Returns string type for a message object of type '<TargetStates>"
  "planner/TargetStates")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetStates)))
  "Returns string type for a message object of type 'TargetStates"
  "planner/TargetStates")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetStates>)))
  "Returns md5sum for a message object of type '<TargetStates>"
  "202d85902896fda1d2900ba1fcceea35")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetStates)))
  "Returns md5sum for a message object of type 'TargetStates"
  "202d85902896fda1d2900ba1fcceea35")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetStates>)))
  "Returns full string definition for message of type '<TargetStates>"
  (cl:format cl:nil "Header header~%float32 target_roll~%float32 target_pitch~%float32 target_yaw~%float32 target_depth~%float32 target_surge~%float32 target_lat~%float32 target_lon~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetStates)))
  "Returns full string definition for message of type 'TargetStates"
  (cl:format cl:nil "Header header~%float32 target_roll~%float32 target_pitch~%float32 target_yaw~%float32 target_depth~%float32 target_surge~%float32 target_lat~%float32 target_lon~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetStates>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetStates>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetStates
    (cl:cons ':header (header msg))
    (cl:cons ':target_roll (target_roll msg))
    (cl:cons ':target_pitch (target_pitch msg))
    (cl:cons ':target_yaw (target_yaw msg))
    (cl:cons ':target_depth (target_depth msg))
    (cl:cons ':target_surge (target_surge msg))
    (cl:cons ':target_lat (target_lat msg))
    (cl:cons ':target_lon (target_lon msg))
))
