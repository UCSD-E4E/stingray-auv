; Auto-generated. Do not edit!


(in-package planner-msg)


;//! \htmlinclude TargetStates.msg.html

(defclass <TargetStates> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (targetRoll
    :reader targetRoll-val
    :initarg :targetRoll
    :type float
    :initform 0.0)
   (targetPitch
    :reader targetPitch-val
    :initarg :targetPitch
    :type float
    :initform 0.0)
   (targetYaw
    :reader targetYaw-val
    :initarg :targetYaw
    :type float
    :initform 0.0)
   (targetDepth
    :reader targetDepth-val
    :initarg :targetDepth
    :type float
    :initform 0.0)
   (targetSurge
    :reader targetSurge-val
    :initarg :targetSurge
    :type float
    :initform 0.0)
   (targetLat
    :reader targetLat-val
    :initarg :targetLat
    :type float
    :initform 0.0)
   (targetLon
    :reader targetLon-val
    :initarg :targetLon
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <TargetStates>) ostream)
  "Serializes a message object of type '<TargetStates>"
  (serialize (slot-value msg 'header) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'targetRoll))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'targetPitch))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'targetYaw))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'targetDepth))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'targetSurge))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'targetLat))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'targetLon))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <TargetStates>) istream)
  "Deserializes a message object of type '<TargetStates>"
  (deserialize (slot-value msg 'header) istream)
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'targetRoll) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'targetPitch) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'targetYaw) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'targetDepth) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'targetSurge) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'targetLat) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'targetLon) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<TargetStates>)))
  "Returns string type for a message object of type '<TargetStates>"
  "planner/TargetStates")
(defmethod md5sum ((type (eql '<TargetStates>)))
  "Returns md5sum for a message object of type '<TargetStates>"
  "f7b3aba5381b2b1eec6dd44956622d0e")
(defmethod message-definition ((type (eql '<TargetStates>)))
  "Returns full string definition for message of type '<TargetStates>"
  (format nil "Header header~%float32 targetRoll~%float32 targetPitch~%float32 targetYaw~%float32 targetDepth~%float32 targetSurge~%float32 targetLat~%float32 targetLon~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <TargetStates>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <TargetStates>))
  "Converts a ROS message object to a list"
  (list '<TargetStates>
    (cons ':header (header-val msg))
    (cons ':targetRoll (targetRoll-val msg))
    (cons ':targetPitch (targetPitch-val msg))
    (cons ':targetYaw (targetYaw-val msg))
    (cons ':targetDepth (targetDepth-val msg))
    (cons ':targetSurge (targetSurge-val msg))
    (cons ':targetLat (targetLat-val msg))
    (cons ':targetLon (targetLon-val msg))
))
