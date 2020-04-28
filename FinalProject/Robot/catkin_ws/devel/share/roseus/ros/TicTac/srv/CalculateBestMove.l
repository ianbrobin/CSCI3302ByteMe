;; Auto-generated. Do not edit!


(when (boundp 'TicTac::CalculateBestMove)
  (if (not (find-package "TICTAC"))
    (make-package "TICTAC"))
  (shadow 'CalculateBestMove (find-package "TICTAC")))
(unless (find-package "TICTAC::CALCULATEBESTMOVE")
  (make-package "TICTAC::CALCULATEBESTMOVE"))
(unless (find-package "TICTAC::CALCULATEBESTMOVEREQUEST")
  (make-package "TICTAC::CALCULATEBESTMOVEREQUEST"))
(unless (find-package "TICTAC::CALCULATEBESTMOVERESPONSE")
  (make-package "TICTAC::CALCULATEBESTMOVERESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass TicTac::CalculateBestMoveRequest
  :super ros::object
  :slots (_input ))

(defmethod TicTac::CalculateBestMoveRequest
  (:init
   (&key
    ((:input __input) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&rest __input)
   (if (keywordp (car __input))
       (send* _input __input)
     (progn
       (if __input (setq _input (car __input)))
       _input)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass TicTac::CalculateBestMoveResponse
  :super ros::object
  :slots (_output ))

(defmethod TicTac::CalculateBestMoveResponse
  (:init
   (&key
    ((:output __output) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _output __output)
   self)
  (:output
   (&rest __output)
   (if (keywordp (car __output))
       (send* _output __output)
     (progn
       (if __output (setq _output (car __output)))
       _output)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass TicTac::CalculateBestMove
  :super ros::object
  :slots ())

(setf (get TicTac::CalculateBestMove :md5sum-) "d8b7e9905e6a815639dc67959117aa3b")
(setf (get TicTac::CalculateBestMove :datatype-) "TicTac/CalculateBestMove")
(setf (get TicTac::CalculateBestMove :request) TicTac::CalculateBestMoveRequest)
(setf (get TicTac::CalculateBestMove :response) TicTac::CalculateBestMoveResponse)

(defmethod TicTac::CalculateBestMoveRequest
  (:response () (instance TicTac::CalculateBestMoveResponse :init)))

(setf (get TicTac::CalculateBestMoveRequest :md5sum-) "d8b7e9905e6a815639dc67959117aa3b")
(setf (get TicTac::CalculateBestMoveRequest :datatype-) "TicTac/CalculateBestMoveRequest")
(setf (get TicTac::CalculateBestMoveRequest :definition-)
      "std_msgs/String input

================================================================================
MSG: std_msgs/String
string data
---
std_msgs/String output


================================================================================
MSG: std_msgs/String
string data
")

(setf (get TicTac::CalculateBestMoveResponse :md5sum-) "d8b7e9905e6a815639dc67959117aa3b")
(setf (get TicTac::CalculateBestMoveResponse :datatype-) "TicTac/CalculateBestMoveResponse")
(setf (get TicTac::CalculateBestMoveResponse :definition-)
      "std_msgs/String input

================================================================================
MSG: std_msgs/String
string data
---
std_msgs/String output


================================================================================
MSG: std_msgs/String
string data
")



(provide :TicTac/CalculateBestMove "d8b7e9905e6a815639dc67959117aa3b")


