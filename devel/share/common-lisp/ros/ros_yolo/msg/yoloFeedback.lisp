; Auto-generated. Do not edit!


(cl:in-package ros_yolo-msg)


;//! \htmlinclude yoloFeedback.msg.html

(cl:defclass <yoloFeedback> (roslisp-msg-protocol:ros-message)
  ((feedback_vector
    :reader feedback_vector
    :initarg :feedback_vector
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass yoloFeedback (<yoloFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <yoloFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'yoloFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_yolo-msg:<yoloFeedback> is deprecated: use ros_yolo-msg:yoloFeedback instead.")))

(cl:ensure-generic-function 'feedback_vector-val :lambda-list '(m))
(cl:defmethod feedback_vector-val ((m <yoloFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_yolo-msg:feedback_vector-val is deprecated.  Use ros_yolo-msg:feedback_vector instead.")
  (feedback_vector m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <yoloFeedback>) ostream)
  "Serializes a message object of type '<yoloFeedback>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'feedback_vector))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'feedback_vector))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <yoloFeedback>) istream)
  "Deserializes a message object of type '<yoloFeedback>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'feedback_vector) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'feedback_vector)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<yoloFeedback>)))
  "Returns string type for a message object of type '<yoloFeedback>"
  "ros_yolo/yoloFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'yoloFeedback)))
  "Returns string type for a message object of type 'yoloFeedback"
  "ros_yolo/yoloFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<yoloFeedback>)))
  "Returns md5sum for a message object of type '<yoloFeedback>"
  "2ee6f212dbc91cfc6dfefacdb2b93a33")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'yoloFeedback)))
  "Returns md5sum for a message object of type 'yoloFeedback"
  "2ee6f212dbc91cfc6dfefacdb2b93a33")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<yoloFeedback>)))
  "Returns full string definition for message of type '<yoloFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%int32[] feedback_vector~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'yoloFeedback)))
  "Returns full string definition for message of type 'yoloFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%int32[] feedback_vector~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <yoloFeedback>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'feedback_vector) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <yoloFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'yoloFeedback
    (cl:cons ':feedback_vector (feedback_vector msg))
))
