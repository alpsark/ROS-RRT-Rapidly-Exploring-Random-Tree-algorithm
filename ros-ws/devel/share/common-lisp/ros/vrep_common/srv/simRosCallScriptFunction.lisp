; Auto-generated. Do not edit!


(cl:in-package vrep_common-srv)


;//! \htmlinclude simRosCallScriptFunction-request.msg.html

(cl:defclass <simRosCallScriptFunction-request> (roslisp-msg-protocol:ros-message)
  ((functionNameAtObjectName
    :reader functionNameAtObjectName
    :initarg :functionNameAtObjectName
    :type cl:string
    :initform "")
   (scriptHandleOrType
    :reader scriptHandleOrType
    :initarg :scriptHandleOrType
    :type cl:integer
    :initform 0)
   (inputInts
    :reader inputInts
    :initarg :inputInts
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (inputFloats
    :reader inputFloats
    :initarg :inputFloats
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (inputStrings
    :reader inputStrings
    :initarg :inputStrings
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (inputBuffer
    :reader inputBuffer
    :initarg :inputBuffer
    :type cl:string
    :initform ""))
)

(cl:defclass simRosCallScriptFunction-request (<simRosCallScriptFunction-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <simRosCallScriptFunction-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'simRosCallScriptFunction-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrep_common-srv:<simRosCallScriptFunction-request> is deprecated: use vrep_common-srv:simRosCallScriptFunction-request instead.")))

(cl:ensure-generic-function 'functionNameAtObjectName-val :lambda-list '(m))
(cl:defmethod functionNameAtObjectName-val ((m <simRosCallScriptFunction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrep_common-srv:functionNameAtObjectName-val is deprecated.  Use vrep_common-srv:functionNameAtObjectName instead.")
  (functionNameAtObjectName m))

(cl:ensure-generic-function 'scriptHandleOrType-val :lambda-list '(m))
(cl:defmethod scriptHandleOrType-val ((m <simRosCallScriptFunction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrep_common-srv:scriptHandleOrType-val is deprecated.  Use vrep_common-srv:scriptHandleOrType instead.")
  (scriptHandleOrType m))

(cl:ensure-generic-function 'inputInts-val :lambda-list '(m))
(cl:defmethod inputInts-val ((m <simRosCallScriptFunction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrep_common-srv:inputInts-val is deprecated.  Use vrep_common-srv:inputInts instead.")
  (inputInts m))

(cl:ensure-generic-function 'inputFloats-val :lambda-list '(m))
(cl:defmethod inputFloats-val ((m <simRosCallScriptFunction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrep_common-srv:inputFloats-val is deprecated.  Use vrep_common-srv:inputFloats instead.")
  (inputFloats m))

(cl:ensure-generic-function 'inputStrings-val :lambda-list '(m))
(cl:defmethod inputStrings-val ((m <simRosCallScriptFunction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrep_common-srv:inputStrings-val is deprecated.  Use vrep_common-srv:inputStrings instead.")
  (inputStrings m))

(cl:ensure-generic-function 'inputBuffer-val :lambda-list '(m))
(cl:defmethod inputBuffer-val ((m <simRosCallScriptFunction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrep_common-srv:inputBuffer-val is deprecated.  Use vrep_common-srv:inputBuffer instead.")
  (inputBuffer m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <simRosCallScriptFunction-request>) ostream)
  "Serializes a message object of type '<simRosCallScriptFunction-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'functionNameAtObjectName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'functionNameAtObjectName))
  (cl:let* ((signed (cl:slot-value msg 'scriptHandleOrType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'inputInts))))
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
   (cl:slot-value msg 'inputInts))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'inputFloats))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'inputFloats))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'inputStrings))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'inputStrings))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'inputBuffer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'inputBuffer))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <simRosCallScriptFunction-request>) istream)
  "Deserializes a message object of type '<simRosCallScriptFunction-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'functionNameAtObjectName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'functionNameAtObjectName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'scriptHandleOrType) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'inputInts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'inputInts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'inputFloats) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'inputFloats)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'inputStrings) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'inputStrings)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'inputBuffer) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'inputBuffer) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<simRosCallScriptFunction-request>)))
  "Returns string type for a service object of type '<simRosCallScriptFunction-request>"
  "vrep_common/simRosCallScriptFunctionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simRosCallScriptFunction-request)))
  "Returns string type for a service object of type 'simRosCallScriptFunction-request"
  "vrep_common/simRosCallScriptFunctionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<simRosCallScriptFunction-request>)))
  "Returns md5sum for a message object of type '<simRosCallScriptFunction-request>"
  "df820c7b6bf8dac295ae340e203b4857")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'simRosCallScriptFunction-request)))
  "Returns md5sum for a message object of type 'simRosCallScriptFunction-request"
  "df820c7b6bf8dac295ae340e203b4857")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<simRosCallScriptFunction-request>)))
  "Returns full string definition for message of type '<simRosCallScriptFunction-request>"
  (cl:format cl:nil "~%~%~%~%string functionNameAtObjectName~%int32 scriptHandleOrType~%int32[] inputInts~%float32[] inputFloats~%string[] inputStrings~%string inputBuffer~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'simRosCallScriptFunction-request)))
  "Returns full string definition for message of type 'simRosCallScriptFunction-request"
  (cl:format cl:nil "~%~%~%~%string functionNameAtObjectName~%int32 scriptHandleOrType~%int32[] inputInts~%float32[] inputFloats~%string[] inputStrings~%string inputBuffer~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <simRosCallScriptFunction-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'functionNameAtObjectName))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'inputInts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'inputFloats) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'inputStrings) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:length (cl:slot-value msg 'inputBuffer))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <simRosCallScriptFunction-request>))
  "Converts a ROS message object to a list"
  (cl:list 'simRosCallScriptFunction-request
    (cl:cons ':functionNameAtObjectName (functionNameAtObjectName msg))
    (cl:cons ':scriptHandleOrType (scriptHandleOrType msg))
    (cl:cons ':inputInts (inputInts msg))
    (cl:cons ':inputFloats (inputFloats msg))
    (cl:cons ':inputStrings (inputStrings msg))
    (cl:cons ':inputBuffer (inputBuffer msg))
))
;//! \htmlinclude simRosCallScriptFunction-response.msg.html

(cl:defclass <simRosCallScriptFunction-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0)
   (outputInts
    :reader outputInts
    :initarg :outputInts
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (outputFloats
    :reader outputFloats
    :initarg :outputFloats
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (outputStrings
    :reader outputStrings
    :initarg :outputStrings
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (outputBuffer
    :reader outputBuffer
    :initarg :outputBuffer
    :type cl:string
    :initform ""))
)

(cl:defclass simRosCallScriptFunction-response (<simRosCallScriptFunction-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <simRosCallScriptFunction-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'simRosCallScriptFunction-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vrep_common-srv:<simRosCallScriptFunction-response> is deprecated: use vrep_common-srv:simRosCallScriptFunction-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <simRosCallScriptFunction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrep_common-srv:result-val is deprecated.  Use vrep_common-srv:result instead.")
  (result m))

(cl:ensure-generic-function 'outputInts-val :lambda-list '(m))
(cl:defmethod outputInts-val ((m <simRosCallScriptFunction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrep_common-srv:outputInts-val is deprecated.  Use vrep_common-srv:outputInts instead.")
  (outputInts m))

(cl:ensure-generic-function 'outputFloats-val :lambda-list '(m))
(cl:defmethod outputFloats-val ((m <simRosCallScriptFunction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrep_common-srv:outputFloats-val is deprecated.  Use vrep_common-srv:outputFloats instead.")
  (outputFloats m))

(cl:ensure-generic-function 'outputStrings-val :lambda-list '(m))
(cl:defmethod outputStrings-val ((m <simRosCallScriptFunction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrep_common-srv:outputStrings-val is deprecated.  Use vrep_common-srv:outputStrings instead.")
  (outputStrings m))

(cl:ensure-generic-function 'outputBuffer-val :lambda-list '(m))
(cl:defmethod outputBuffer-val ((m <simRosCallScriptFunction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vrep_common-srv:outputBuffer-val is deprecated.  Use vrep_common-srv:outputBuffer instead.")
  (outputBuffer m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <simRosCallScriptFunction-response>) ostream)
  "Serializes a message object of type '<simRosCallScriptFunction-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'outputInts))))
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
   (cl:slot-value msg 'outputInts))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'outputFloats))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'outputFloats))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'outputStrings))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'outputStrings))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'outputBuffer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'outputBuffer))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <simRosCallScriptFunction-response>) istream)
  "Deserializes a message object of type '<simRosCallScriptFunction-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'outputInts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'outputInts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'outputFloats) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'outputFloats)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'outputStrings) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'outputStrings)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'outputBuffer) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'outputBuffer) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<simRosCallScriptFunction-response>)))
  "Returns string type for a service object of type '<simRosCallScriptFunction-response>"
  "vrep_common/simRosCallScriptFunctionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simRosCallScriptFunction-response)))
  "Returns string type for a service object of type 'simRosCallScriptFunction-response"
  "vrep_common/simRosCallScriptFunctionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<simRosCallScriptFunction-response>)))
  "Returns md5sum for a message object of type '<simRosCallScriptFunction-response>"
  "df820c7b6bf8dac295ae340e203b4857")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'simRosCallScriptFunction-response)))
  "Returns md5sum for a message object of type 'simRosCallScriptFunction-response"
  "df820c7b6bf8dac295ae340e203b4857")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<simRosCallScriptFunction-response>)))
  "Returns full string definition for message of type '<simRosCallScriptFunction-response>"
  (cl:format cl:nil "int32 result~%int32[] outputInts~%float32[] outputFloats~%string[] outputStrings~%string outputBuffer~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'simRosCallScriptFunction-response)))
  "Returns full string definition for message of type 'simRosCallScriptFunction-response"
  (cl:format cl:nil "int32 result~%int32[] outputInts~%float32[] outputFloats~%string[] outputStrings~%string outputBuffer~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <simRosCallScriptFunction-response>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'outputInts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'outputFloats) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'outputStrings) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:length (cl:slot-value msg 'outputBuffer))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <simRosCallScriptFunction-response>))
  "Converts a ROS message object to a list"
  (cl:list 'simRosCallScriptFunction-response
    (cl:cons ':result (result msg))
    (cl:cons ':outputInts (outputInts msg))
    (cl:cons ':outputFloats (outputFloats msg))
    (cl:cons ':outputStrings (outputStrings msg))
    (cl:cons ':outputBuffer (outputBuffer msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'simRosCallScriptFunction)))
  'simRosCallScriptFunction-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'simRosCallScriptFunction)))
  'simRosCallScriptFunction-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'simRosCallScriptFunction)))
  "Returns string type for a service object of type '<simRosCallScriptFunction>"
  "vrep_common/simRosCallScriptFunction")