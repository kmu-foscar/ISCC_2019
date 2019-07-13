; Auto-generated. Do not edit!


(cl:in-package race-msg)


;//! \htmlinclude drive_values.msg.html

(cl:defclass <drive_values> (roslisp-msg-protocol:ros-message)
  ((throttle
    :reader throttle
    :initarg :throttle
    :type cl:fixnum
    :initform 0)
   (steering
    :reader steering
    :initarg :steering
    :type cl:fixnum
    :initform 0))
)

(cl:defclass drive_values (<drive_values>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <drive_values>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'drive_values)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name race-msg:<drive_values> is deprecated: use race-msg:drive_values instead.")))

(cl:ensure-generic-function 'throttle-val :lambda-list '(m))
(cl:defmethod throttle-val ((m <drive_values>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader race-msg:throttle-val is deprecated.  Use race-msg:throttle instead.")
  (throttle m))

(cl:ensure-generic-function 'steering-val :lambda-list '(m))
(cl:defmethod steering-val ((m <drive_values>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader race-msg:steering-val is deprecated.  Use race-msg:steering instead.")
  (steering m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <drive_values>) ostream)
  "Serializes a message object of type '<drive_values>"
  (cl:let* ((signed (cl:slot-value msg 'throttle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'steering)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <drive_values>) istream)
  "Deserializes a message object of type '<drive_values>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'throttle) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'steering) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<drive_values>)))
  "Returns string type for a message object of type '<drive_values>"
  "race/drive_values")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'drive_values)))
  "Returns string type for a message object of type 'drive_values"
  "race/drive_values")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<drive_values>)))
  "Returns md5sum for a message object of type '<drive_values>"
  "0194e179a126494f7021e4f23090fd6b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'drive_values)))
  "Returns md5sum for a message object of type 'drive_values"
  "0194e179a126494f7021e4f23090fd6b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<drive_values>)))
  "Returns full string definition for message of type '<drive_values>"
  (cl:format cl:nil "int16 throttle~%int16 steering~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'drive_values)))
  "Returns full string definition for message of type 'drive_values"
  (cl:format cl:nil "int16 throttle~%int16 steering~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <drive_values>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <drive_values>))
  "Converts a ROS message object to a list"
  (cl:list 'drive_values
    (cl:cons ':throttle (throttle msg))
    (cl:cons ':steering (steering msg))
))
