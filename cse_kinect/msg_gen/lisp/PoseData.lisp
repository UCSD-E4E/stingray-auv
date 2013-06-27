; Auto-generated. Do not edit!


(cl:in-package cse_kinect-msg)


;//! \htmlinclude PoseData.msg.html

(cl:defclass <PoseData> (roslisp-msg-protocol:ros-message)
  ((pose1
    :reader pose1
    :initarg :pose1
    :type cl:boolean
    :initform cl:nil)
   (pose2
    :reader pose2
    :initarg :pose2
    :type cl:boolean
    :initform cl:nil)
   (lVoila
    :reader lVoila
    :initarg :lVoila
    :type cl:boolean
    :initform cl:nil)
   (rVoila
    :reader rVoila
    :initarg :rVoila
    :type cl:boolean
    :initform cl:nil)
   (flat
    :reader flat
    :initarg :flat
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PoseData (<PoseData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PoseData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PoseData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cse_kinect-msg:<PoseData> is deprecated: use cse_kinect-msg:PoseData instead.")))

(cl:ensure-generic-function 'pose1-val :lambda-list '(m))
(cl:defmethod pose1-val ((m <PoseData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cse_kinect-msg:pose1-val is deprecated.  Use cse_kinect-msg:pose1 instead.")
  (pose1 m))

(cl:ensure-generic-function 'pose2-val :lambda-list '(m))
(cl:defmethod pose2-val ((m <PoseData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cse_kinect-msg:pose2-val is deprecated.  Use cse_kinect-msg:pose2 instead.")
  (pose2 m))

(cl:ensure-generic-function 'lVoila-val :lambda-list '(m))
(cl:defmethod lVoila-val ((m <PoseData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cse_kinect-msg:lVoila-val is deprecated.  Use cse_kinect-msg:lVoila instead.")
  (lVoila m))

(cl:ensure-generic-function 'rVoila-val :lambda-list '(m))
(cl:defmethod rVoila-val ((m <PoseData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cse_kinect-msg:rVoila-val is deprecated.  Use cse_kinect-msg:rVoila instead.")
  (rVoila m))

(cl:ensure-generic-function 'flat-val :lambda-list '(m))
(cl:defmethod flat-val ((m <PoseData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cse_kinect-msg:flat-val is deprecated.  Use cse_kinect-msg:flat instead.")
  (flat m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PoseData>) ostream)
  "Serializes a message object of type '<PoseData>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pose1) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pose2) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'lVoila) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'rVoila) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flat) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PoseData>) istream)
  "Deserializes a message object of type '<PoseData>"
    (cl:setf (cl:slot-value msg 'pose1) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'pose2) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'lVoila) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'rVoila) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'flat) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PoseData>)))
  "Returns string type for a message object of type '<PoseData>"
  "cse_kinect/PoseData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseData)))
  "Returns string type for a message object of type 'PoseData"
  "cse_kinect/PoseData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PoseData>)))
  "Returns md5sum for a message object of type '<PoseData>"
  "1261c601270ddadfdc36b3c644334171")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PoseData)))
  "Returns md5sum for a message object of type 'PoseData"
  "1261c601270ddadfdc36b3c644334171")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PoseData>)))
  "Returns full string definition for message of type '<PoseData>"
  (cl:format cl:nil "bool pose1~%bool pose2~%bool lVoila~%bool rVoila~%bool flat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PoseData)))
  "Returns full string definition for message of type 'PoseData"
  (cl:format cl:nil "bool pose1~%bool pose2~%bool lVoila~%bool rVoila~%bool flat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PoseData>))
  (cl:+ 0
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PoseData>))
  "Converts a ROS message object to a list"
  (cl:list 'PoseData
    (cl:cons ':pose1 (pose1 msg))
    (cl:cons ':pose2 (pose2 msg))
    (cl:cons ':lVoila (lVoila msg))
    (cl:cons ':rVoila (rVoila msg))
    (cl:cons ':flat (flat msg))
))
