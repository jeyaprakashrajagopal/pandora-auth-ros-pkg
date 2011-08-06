; Auto-generated. Do not edit!


(in-package vision_communications-msg)


;//! \htmlinclude lineColorMsg.msg.html

(defclass <lineColorMsg> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (fromAngle
    :reader fromAngle-val
    :initarg :fromAngle
    :type float
    :initform 0.0)
   (toAngle
    :reader toAngle-val
    :initarg :toAngle
    :type float
    :initform 0.0)
   (color
    :reader color-val
    :initarg :color
    :type fixnum
    :initform 0)
   (orientation
    :reader orientation-val
    :initarg :orientation
    :type fixnum
    :initform 0))
)
(defmethod symbol-codes ((msg-type (eql '<lineColorMsg>)))
  "Constants for message type '<lineColorMsg>"
  '((:COLOR_YELLOW . 1)
    (:COLOR_RED . 2)
    (:COLOR_ORANGE . 3)
    (:ORIENTATION_NONE . 0)
    (:ORIENTATION_HORIZONTAL . 1)
    (:ORIENTATION_VERTICAL . 2))
)
(defmethod serialize ((msg <lineColorMsg>) ostream)
  "Serializes a message object of type '<lineColorMsg>"
  (serialize (slot-value msg 'header) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'fromAngle))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'toAngle))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (write-byte (ldb (byte 8 0) (slot-value msg 'color)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'orientation)) ostream)
)
(defmethod deserialize ((msg <lineColorMsg>) istream)
  "Deserializes a message object of type '<lineColorMsg>"
  (deserialize (slot-value msg 'header) istream)
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'fromAngle) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'toAngle) (roslisp-utils:decode-single-float-bits bits)))
  (setf (ldb (byte 8 0) (slot-value msg 'color)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'orientation)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<lineColorMsg>)))
  "Returns string type for a message object of type '<lineColorMsg>"
  "vision_communications/lineColorMsg")
(defmethod md5sum ((type (eql '<lineColorMsg>)))
  "Returns md5sum for a message object of type '<lineColorMsg>"
  "ec6e84360b96d21a4c616a0d272c144b")
(defmethod message-definition ((type (eql '<lineColorMsg>)))
  "Returns full string definition for message of type '<lineColorMsg>"
  (format nil "Header header~%float32 fromAngle~%float32 toAngle~%uint8 color~%uint8 orientation~%~%uint8 COLOR_YELLOW=1~%uint8 COLOR_RED=2~%uint8 COLOR_ORANGE=3~%~%uint8 ORIENTATION_NONE=0~%uint8 ORIENTATION_HORIZONTAL=1~%uint8 ORIENTATION_VERTICAL=2~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <lineColorMsg>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
     4
     1
     1
))
(defmethod ros-message-to-list ((msg <lineColorMsg>))
  "Converts a ROS message object to a list"
  (list '<lineColorMsg>
    (cons ':header (header-val msg))
    (cons ':fromAngle (fromAngle-val msg))
    (cons ':toAngle (toAngle-val msg))
    (cons ':color (color-val msg))
    (cons ':orientation (orientation-val msg))
))
