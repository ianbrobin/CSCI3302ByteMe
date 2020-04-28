
(cl:in-package :asdf)

(defsystem "TicTac-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CalculateBestMove" :depends-on ("_package_CalculateBestMove"))
    (:file "_package_CalculateBestMove" :depends-on ("_package"))
  ))