; Auto-generated. Do not edit!


(in-package vision_communications-msg)


;//! \htmlinclude hazmatIdentificationMsg.msg.html

(defclass <hazmatIdentificationMsg> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (x
    :reader x-val
    :initarg :x
    :type integer
    :initform 0)
   (y
    :reader y-val
    :initarg :y
    :type integer
    :initform 0)
   (z
    :reader z-val
    :initarg :z
    :type integer
    :initform 0)
   (patternType
    :reader patternType-val
    :initarg :patternType
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <hazmatIdentificationMsg>) ostream)
  "Serializes a message object of type '<hazmatIdentificationMsg>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'x)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'x)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'x)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'x)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'y)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'y)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'y)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'y)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'z)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'z)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'z)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'z)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'patternType)) ostream)
)
(defmethod deserialize ((msg <hazmatIdentificationMsg>) istream)
  "Deserializes a message object of type '<hazmatIdentificationMsg>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'x)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'y)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'z)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'z)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'z)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'z)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'patternType)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<hazmatIdentificationMsg>)))
  "Returns string type for a message object of type '<hazmatIdentificationMsg>"
  "vision_communications/hazmatIdentificationMsg")
(defmethod md5sum ((type (eql '<hazmatIdentificationMsg>)))
  "Returns md5sum for a message object of type '<hazmatIdentificationMsg>"
  "e4c10eb4a5b14fe52318fe5fd9f753f4")
(defmethod message-definition ((type (eql '<hazmatIdentificationMsg>)))
  "Returns full string definition for message of type '<hazmatIdentificationMsg>"
  (format nil "Header header~%int32 x~%int32 y~%int32 z~%uint8 patternType~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <hazmatIdentificationMsg>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
     4
     4
     1
))
(defmethod ros-message-to-list ((msg <hazmatIdentificationMsg>))
  "Converts a ROS message object to a list"
  (list '<hazmatIdentificationMsg>
    (cons ':header (header-val msg))
    (cons ':x (x-val msg))
    (cons ':y (y-val msg))
    (cons ':z (z-val msg))
    (cons ':patternType (patternType-val msg))
))
