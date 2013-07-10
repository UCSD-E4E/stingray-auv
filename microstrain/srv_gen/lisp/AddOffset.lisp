; Auto-generated. Do not edit!


(cl:in-package microstrain-srv)


;//! \htmlinclude AddOffset-request.msg.html

(cl:defclass <AddOffset-request> (roslisp-msg-protocol:ros-message)
  ((add_offset
    :reader add_offset
    :initarg :add_offset
    :type cl:float
    :initform 0.0))
)

(cl:defclass AddOffset-request (<AddOffset-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddOffset-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddOffset-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name microstrain-srv:<AddOffset-request> is deprecated: use microstrain-srv:AddOffset-request instead.")))

(cl:ensure-generic-function 'add_offset-val :lambda-list '(m))
(cl:defmethod add_offset-val ((m <AddOffset-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader microstrain-srv:add_offset-val is deprecated.  Use microstrain-srv:add_offset instead.")
  (add_offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddOffset-request>) ostream)
  "Serializes a message object of type '<AddOffset-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'add_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddOffset-request>) istream)
  "Deserializes a message object of type '<AddOffset-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'add_offset) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddOffset-request>)))
  "Returns string type for a service object of type '<AddOffset-request>"
  "microstrain/AddOffsetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddOffset-request)))
  "Returns string type for a service object of type 'AddOffset-request"
  "microstrain/AddOffsetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddOffset-request>)))
  "Returns md5sum for a message object of type '<AddOffset-request>"
  "f5dcf1246c1a25fcc69616e9d14c1482")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddOffset-request)))
  "Returns md5sum for a message object of type 'AddOffset-request"
  "f5dcf1246c1a25fcc69616e9d14c1482")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddOffset-request>)))
  "Returns full string definition for message of type '<AddOffset-request>"
  (cl:format cl:nil "float64 add_offset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddOffset-request)))
  "Returns full string definition for message of type 'AddOffset-request"
  (cl:format cl:nil "float64 add_offset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddOffset-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddOffset-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddOffset-request
    (cl:cons ':add_offset (add_offset msg))
))
;//! \htmlinclude AddOffset-response.msg.html

(cl:defclass <AddOffset-response> (roslisp-msg-protocol:ros-message)
  ((total_offset
    :reader total_offset
    :initarg :total_offset
    :type cl:float
    :initform 0.0))
)

(cl:defclass AddOffset-response (<AddOffset-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddOffset-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddOffset-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name microstrain-srv:<AddOffset-response> is deprecated: use microstrain-srv:AddOffset-response instead.")))

(cl:ensure-generic-function 'total_offset-val :lambda-list '(m))
(cl:defmethod total_offset-val ((m <AddOffset-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader microstrain-srv:total_offset-val is deprecated.  Use microstrain-srv:total_offset instead.")
  (total_offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddOffset-response>) ostream)
  "Serializes a message object of type '<AddOffset-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'total_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddOffset-response>) istream)
  "Deserializes a message object of type '<AddOffset-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_offset) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddOffset-response>)))
  "Returns string type for a service object of type '<AddOffset-response>"
  "microstrain/AddOffsetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddOffset-response)))
  "Returns string type for a service object of type 'AddOffset-response"
  "microstrain/AddOffsetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddOffset-response>)))
  "Returns md5sum for a message object of type '<AddOffset-response>"
  "f5dcf1246c1a25fcc69616e9d14c1482")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddOffset-response)))
  "Returns md5sum for a message object of type 'AddOffset-response"
  "f5dcf1246c1a25fcc69616e9d14c1482")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddOffset-response>)))
  "Returns full string definition for message of type '<AddOffset-response>"
  (cl:format cl:nil "float64 total_offset~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddOffset-response)))
  "Returns full string definition for message of type 'AddOffset-response"
  (cl:format cl:nil "float64 total_offset~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddOffset-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddOffset-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddOffset-response
    (cl:cons ':total_offset (total_offset msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddOffset)))
  'AddOffset-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddOffset)))
  'AddOffset-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddOffset)))
  "Returns string type for a service object of type '<AddOffset>"
  "microstrain/AddOffset")