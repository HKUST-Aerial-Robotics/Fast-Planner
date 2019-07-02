
(cl:in-package :asdf)

(defsystem "quadrotor_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Serial" :depends-on ("_package_Serial"))
    (:file "_package_Serial" :depends-on ("_package"))
    (:file "AuxCommand" :depends-on ("_package_AuxCommand"))
    (:file "_package_AuxCommand" :depends-on ("_package"))
    (:file "PositionCommand" :depends-on ("_package_PositionCommand"))
    (:file "_package_PositionCommand" :depends-on ("_package"))
    (:file "Corrections" :depends-on ("_package_Corrections"))
    (:file "_package_Corrections" :depends-on ("_package"))
    (:file "StatusData" :depends-on ("_package_StatusData"))
    (:file "_package_StatusData" :depends-on ("_package"))
    (:file "SO3Command" :depends-on ("_package_SO3Command"))
    (:file "_package_SO3Command" :depends-on ("_package"))
    (:file "PPROutputData" :depends-on ("_package_PPROutputData"))
    (:file "_package_PPROutputData" :depends-on ("_package"))
    (:file "TRPYCommand" :depends-on ("_package_TRPYCommand"))
    (:file "_package_TRPYCommand" :depends-on ("_package"))
    (:file "OutputData" :depends-on ("_package_OutputData"))
    (:file "_package_OutputData" :depends-on ("_package"))
    (:file "Gains" :depends-on ("_package_Gains"))
    (:file "_package_Gains" :depends-on ("_package"))
  ))