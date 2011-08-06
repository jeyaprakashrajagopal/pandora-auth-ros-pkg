; Auto-generated. Do not edit!


(in-package vision_communications-msg)


;//! \htmlinclude victimIdentificationDirectionMsg.msg.html

(defclass <victimIdentificationDirectionMsg> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (x
    :reader x-val
    :initarg :x
    :type float
    :initform 0.0)
   (y
    :reader y-val
    :initarg :y
    :type float
    :initform 0.0)
   (probability
    :reader probability-val
    :initarg :probability
    :type float
    :initform 0.0)
   (type
    :reader type-val
    :initarg :type
    :type fixnum
    :initform 0))
)
(defmethod symbol-codes ((msg-type (eql '<victimIdentificationDirectionMsg>)))
  "Constants for message type '<victimIdentificationDirectionMsg>"
  '((:FACE . 1)
    (:HOLE . 2)
    (:SKIN . 3)
    (:MOTION . 4))
)
(defmethod serialize ((msg <victimIdentificationDirectionMsg>) ostream)
  "Serializes a message object of type '<victimIdentificationDirectionMsg>"
  (serialize (slot-value msg 'header) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'x))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'y))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'probability))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (write-byte (ldb (byte 8 0) (slot-value msg 'type)) ostream)
)
(defmethod deserialize ((msg <victimIdentificationDirectionMsg>) istream)
  "Deserializes a message object of type '<victimIdentificationDirectionMsg>"
  (deserialize (slot-value msg 'header) istream)
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'probability) (roslisp-utils:decode-single-float-bits bits)))
  (setf (ldb (byte 8 0) (slot-value msg 'type)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<victimIdentificationDirectionMsg>)))
  "Returns string type for a message object of type '<victimIdentificationDirectionMsg>"
  "vision_communications/victimIdentificationDirectionMsg")
(defmethod md5sum ((type (eql '<victimIdentificationDirectionMsg>)))
  "Returns md5sum for a message object of type '<victimIdentificationDirectionMsg>"
  "cf3a0cc2bb7c3d1bd69d8ec92dba5242")
(defmethod message-definition ((type (eql '<victimIdentificationDirectionMsg>)))
  "Returns full string definition for message of type '<victimIdentificationDirectionMsg>"
  (format nil "Header header~%float32 x~%float32 y~%float32 probability~%uint8 type~%~%uint8 FACE=1~%uint8 HOLE=2~%uint8 SKIN=3~%uint8 MOTION=4~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <victimIdentificationDirectionMsg>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
     4
     4
     1
))
(defmethod ros-message-to-list ((msg <victimIdentificationDirectionMsg>))
  "Converts a ROS message object to a list"
  (list '<victimIdentificationDirectionMsg>
    (cons ':header (header-val msg))
    (cons ':x (x-val msg))
    (cons ':y (y-val msg))
    (cons ':probability (probability-val msg))
    (cons ':type (type-val msg))
))
