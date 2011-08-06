
(in-package :asdf)

(defsystem "vision_communications-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "victimIdentificationPositionMsg" :depends-on ("_package"))
    (:file "_package_victimIdentificationPositionMsg" :depends-on ("_package"))
    (:file "hazmatIdentificationMsg" :depends-on ("_package"))
    (:file "_package_hazmatIdentificationMsg" :depends-on ("_package"))
    (:file "victimIdentificationDirectionMsg" :depends-on ("_package"))
    (:file "_package_victimIdentificationDirectionMsg" :depends-on ("_package"))
    (:file "lineColorMsg" :depends-on ("_package"))
    (:file "_package_lineColorMsg" :depends-on ("_package"))
    ))
