; Auto-generated. Do not edit!


(cl:in-package sbBTA252-msg)


;//! \htmlinclude SBBTA252Data.msg.html

(cl:defclass <SBBTA252Data> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (current
    :reader current
    :initarg :current
    :type cl:float
    :initform 0.0)
   (motor_temp
    :reader motor_temp
    :initarg :motor_temp
    :type cl:float
    :initform 0.0)
   (controller_temp
    :reader controller_temp
    :initarg :controller_temp
    :type cl:float
    :initform 0.0)
   (controller_volt
    :reader controller_volt
    :initarg :controller_volt
    :type cl:float
    :initform 0.0)
   (water
    :reader water
    :initarg :water
    :type cl:float
    :initform 0.0))
)

(cl:defclass SBBTA252Data (<SBBTA252Data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SBBTA252Data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SBBTA252Data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sbBTA252-msg:<SBBTA252Data> is deprecated: use sbBTA252-msg:SBBTA252Data instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <SBBTA252Data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbBTA252-msg:speed-val is deprecated.  Use sbBTA252-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <SBBTA252Data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbBTA252-msg:current-val is deprecated.  Use sbBTA252-msg:current instead.")
  (current m))

(cl:ensure-generic-function 'motor_temp-val :lambda-list '(m))
(cl:defmethod motor_temp-val ((m <SBBTA252Data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbBTA252-msg:motor_temp-val is deprecated.  Use sbBTA252-msg:motor_temp instead.")
  (motor_temp m))

(cl:ensure-generic-function 'controller_temp-val :lambda-list '(m))
(cl:defmethod controller_temp-val ((m <SBBTA252Data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbBTA252-msg:controller_temp-val is deprecated.  Use sbBTA252-msg:controller_temp instead.")
  (controller_temp m))

(cl:ensure-generic-function 'controller_volt-val :lambda-list '(m))
(cl:defmethod controller_volt-val ((m <SBBTA252Data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbBTA252-msg:controller_volt-val is deprecated.  Use sbBTA252-msg:controller_volt instead.")
  (controller_volt m))

(cl:ensure-generic-function 'water-val :lambda-list '(m))
(cl:defmethod water-val ((m <SBBTA252Data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbBTA252-msg:water-val is deprecated.  Use sbBTA252-msg:water instead.")
  (water m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SBBTA252Data>) ostream)
  "Serializes a message object of type '<SBBTA252Data>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motor_temp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'controller_temp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'controller_volt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'water))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SBBTA252Data>) istream)
  "Deserializes a message object of type '<SBBTA252Data>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor_temp) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'controller_temp) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'controller_volt) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'water) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SBBTA252Data>)))
  "Returns string type for a message object of type '<SBBTA252Data>"
  "sbBTA252/SBBTA252Data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SBBTA252Data)))
  "Returns string type for a message object of type 'SBBTA252Data"
  "sbBTA252/SBBTA252Data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SBBTA252Data>)))
  "Returns md5sum for a message object of type '<SBBTA252Data>"
  "2681e203ad008d492c8f38edd7805fd9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SBBTA252Data)))
  "Returns md5sum for a message object of type 'SBBTA252Data"
  "2681e203ad008d492c8f38edd7805fd9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SBBTA252Data>)))
  "Returns full string definition for message of type '<SBBTA252Data>"
  (cl:format cl:nil "float32 speed~%float32 current~%float32 motor_temp~%float32 controller_temp~%float32 controller_volt~%float32 water~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SBBTA252Data)))
  "Returns full string definition for message of type 'SBBTA252Data"
  (cl:format cl:nil "float32 speed~%float32 current~%float32 motor_temp~%float32 controller_temp~%float32 controller_volt~%float32 water~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SBBTA252Data>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SBBTA252Data>))
  "Converts a ROS message object to a list"
  (cl:list 'SBBTA252Data
    (cl:cons ':speed (speed msg))
    (cl:cons ':current (current msg))
    (cl:cons ':motor_temp (motor_temp msg))
    (cl:cons ':controller_temp (controller_temp msg))
    (cl:cons ':controller_volt (controller_volt msg))
    (cl:cons ':water (water msg))
))
