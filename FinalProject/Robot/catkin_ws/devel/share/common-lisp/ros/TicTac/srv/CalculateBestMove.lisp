; Auto-generated. Do not edit!


(cl:in-package TicTac-srv)


;//! \htmlinclude CalculateBestMove-request.msg.html

(cl:defclass <CalculateBestMove-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass CalculateBestMove-request (<CalculateBestMove-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalculateBestMove-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalculateBestMove-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name TicTac-srv:<CalculateBestMove-request> is deprecated: use TicTac-srv:CalculateBestMove-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <CalculateBestMove-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader TicTac-srv:input-val is deprecated.  Use TicTac-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalculateBestMove-request>) ostream)
  "Serializes a message object of type '<CalculateBestMove-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalculateBestMove-request>) istream)
  "Deserializes a message object of type '<CalculateBestMove-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalculateBestMove-request>)))
  "Returns string type for a service object of type '<CalculateBestMove-request>"
  "TicTac/CalculateBestMoveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalculateBestMove-request)))
  "Returns string type for a service object of type 'CalculateBestMove-request"
  "TicTac/CalculateBestMoveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalculateBestMove-request>)))
  "Returns md5sum for a message object of type '<CalculateBestMove-request>"
  "d8b7e9905e6a815639dc67959117aa3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalculateBestMove-request)))
  "Returns md5sum for a message object of type 'CalculateBestMove-request"
  "d8b7e9905e6a815639dc67959117aa3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalculateBestMove-request>)))
  "Returns full string definition for message of type '<CalculateBestMove-request>"
  (cl:format cl:nil "std_msgs/String input~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalculateBestMove-request)))
  "Returns full string definition for message of type 'CalculateBestMove-request"
  (cl:format cl:nil "std_msgs/String input~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalculateBestMove-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalculateBestMove-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CalculateBestMove-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude CalculateBestMove-response.msg.html

(cl:defclass <CalculateBestMove-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass CalculateBestMove-response (<CalculateBestMove-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalculateBestMove-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalculateBestMove-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name TicTac-srv:<CalculateBestMove-response> is deprecated: use TicTac-srv:CalculateBestMove-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <CalculateBestMove-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader TicTac-srv:output-val is deprecated.  Use TicTac-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalculateBestMove-response>) ostream)
  "Serializes a message object of type '<CalculateBestMove-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalculateBestMove-response>) istream)
  "Deserializes a message object of type '<CalculateBestMove-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalculateBestMove-response>)))
  "Returns string type for a service object of type '<CalculateBestMove-response>"
  "TicTac/CalculateBestMoveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalculateBestMove-response)))
  "Returns string type for a service object of type 'CalculateBestMove-response"
  "TicTac/CalculateBestMoveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalculateBestMove-response>)))
  "Returns md5sum for a message object of type '<CalculateBestMove-response>"
  "d8b7e9905e6a815639dc67959117aa3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalculateBestMove-response)))
  "Returns md5sum for a message object of type 'CalculateBestMove-response"
  "d8b7e9905e6a815639dc67959117aa3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalculateBestMove-response>)))
  "Returns full string definition for message of type '<CalculateBestMove-response>"
  (cl:format cl:nil "std_msgs/String output~%~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalculateBestMove-response)))
  "Returns full string definition for message of type 'CalculateBestMove-response"
  (cl:format cl:nil "std_msgs/String output~%~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalculateBestMove-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalculateBestMove-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CalculateBestMove-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CalculateBestMove)))
  'CalculateBestMove-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CalculateBestMove)))
  'CalculateBestMove-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalculateBestMove)))
  "Returns string type for a service object of type '<CalculateBestMove>"
  "TicTac/CalculateBestMove")