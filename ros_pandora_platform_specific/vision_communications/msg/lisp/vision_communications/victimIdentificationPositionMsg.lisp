; Auto-generated. Do not edit!


(in-package vision_communications-msg)


;//! \htmlinclude victimIdentificationPositionMsg.msg.html

(defclass <victimIdentificationPositionMsg> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (position
    :reader position-val
    :initarg :position
    :type (vector integer)
   :initform (make-array 3 :element-type 'integer :initial-element 0))
   (normalVector
    :reader normalVector-val
    :initarg :normalVector
    :type (vector float)
   :initform (make-array 3 :element-type 'float :initial-element 0.0)))
)
(defmethod serialize ((msg <victimIdentificationPositionMsg>) ostream)
  "Serializes a message object of type '<victimIdentificationPositionMsg>"
  (serialize (slot-value msg 'header) ostream)
    (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream)
  (write-byte (ldb (byte 8 8) ele) ostream)
  (write-byte (ldb (byte 8 16) ele) ostream)
  (write-byte (ldb (byte 8 24) ele) ostream))(slot-value msg 'position))
    (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))(slot-value msg 'normalVector))
)
(defmethod deserialize ((msg <victimIdentificationPositionMsg>) istream)
  "Deserializes a message object of type '<victimIdentificationPositionMsg>"
  (deserialize (slot-value msg 'header) istream)
  (setf (slot-value msg 'position) (make-array 3))
  (let ((vals (slot-value msg 'position)))
    (dotimes (i 3)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 8) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 16) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 24) (aref vals i)) (read-byte istream))))
  (setf (slot-value msg 'normalVector) (make-array 3))
  (let ((vals (slot-value msg 'normalVector)))
    (dotimes (i 3)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<victimIdentificationPositionMsg>)))
  "Returns string type for a message object of type '<victimIdentificationPositionMsg>"
  "vision_communications/victimIdentificationPositionMsg")
(defmethod md5sum ((type (eql '<victimIdentificationPositionMsg>)))
  "Returns md5sum for a message object of type '<victimIdentificationPositionMsg>"
  "e25b30f74ebf3e7541112b21c6d87cb4")
(defmethod message-definition ((type (eql '<victimIdentificationPositionMsg>)))
  "Returns full string definition for message of type '<victimIdentificationPositionMsg>"
  (format nil "Header header~%int32[3] position~%float32[3] normalVector~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <victimIdentificationPositionMsg>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     0 (reduce #'+ (slot-value msg 'position) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     0 (reduce #'+ (slot-value msg 'normalVector) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
))
(defmethod ros-message-to-list ((msg <victimIdentificationPositionMsg>))
  "Converts a ROS message object to a list"
  (list '<victimIdentificationPositionMsg>
    (cons ':header (header-val msg))
    (cons ':position (position-val msg))
    (cons ':normalVector (normalVector-val msg))
))
