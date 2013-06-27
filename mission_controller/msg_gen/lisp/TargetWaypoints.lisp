; Auto-generated. Do not edit!


(cl:in-package mission_controller-msg)


;//! \htmlinclude TargetWaypoints.msg.html

(cl:defclass <TargetWaypoints> (roslisp-msg-protocol:ros-message)
  ((target_surge
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
    :initform 0.0)
   (target_depth
    :reader target_depth
    :initarg :target_depth
    :type cl:float
    :initform 0.0))
)

(cl:defclass TargetWaypoints (<TargetWaypoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetWaypoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetWaypoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mission_controller-msg:<TargetWaypoints> is deprecated: use mission_controller-msg:TargetWaypoints instead.")))

(cl:ensure-generic-function 'target_surge-val :lambda-list '(m))
(cl:defmethod target_surge-val ((m <TargetWaypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_controller-msg:target_surge-val is deprecated.  Use mission_controller-msg:target_surge instead.")
  (target_surge m))

(cl:ensure-generic-function 'target_lat-val :lambda-list '(m))
(cl:defmethod target_lat-val ((m <TargetWaypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_controller-msg:target_lat-val is deprecated.  Use mission_controller-msg:target_lat instead.")
  (target_lat m))

(cl:ensure-generic-function 'target_lon-val :lambda-list '(m))
(cl:defmethod target_lon-val ((m <TargetWaypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_controller-msg:target_lon-val is deprecated.  Use mission_controller-msg:target_lon instead.")
  (target_lon m))

(cl:ensure-generic-function 'target_depth-val :lambda-list '(m))
(cl:defmethod target_depth-val ((m <TargetWaypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_controller-msg:target_depth-val is deprecated.  Use mission_controller-msg:target_depth instead.")
  (target_depth m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetWaypoints>) ostream)
  "Serializes a message object of type '<TargetWaypoints>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'target_surge))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'target_lat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'target_lon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'target_depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetWaypoints>) istream)
  "Deserializes a message object of type '<TargetWaypoints>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_surge) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_lat) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_lon) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_depth) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetWaypoints>)))
  "Returns string type for a message object of type '<TargetWaypoints>"
  "mission_controller/TargetWaypoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetWaypoints)))
  "Returns string type for a message object of type 'TargetWaypoints"
  "mission_controller/TargetWaypoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetWaypoints>)))
  "Returns md5sum for a message object of type '<TargetWaypoints>"
  "6ef3b92c6a5c0d62a34442009ffdad56")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetWaypoints)))
  "Returns md5sum for a message object of type 'TargetWaypoints"
  "6ef3b92c6a5c0d62a34442009ffdad56")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetWaypoints>)))
  "Returns full string definition for message of type '<TargetWaypoints>"
  (cl:format cl:nil "float64 target_surge~%float64 target_lat~%float64 target_lon~%float64 target_depth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetWaypoints)))
  "Returns full string definition for message of type 'TargetWaypoints"
  (cl:format cl:nil "float64 target_surge~%float64 target_lat~%float64 target_lon~%float64 target_depth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetWaypoints>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetWaypoints>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetWaypoints
    (cl:cons ':target_surge (target_surge msg))
    (cl:cons ':target_lat (target_lat msg))
    (cl:cons ':target_lon (target_lon msg))
    (cl:cons ':target_depth (target_depth msg))
))
