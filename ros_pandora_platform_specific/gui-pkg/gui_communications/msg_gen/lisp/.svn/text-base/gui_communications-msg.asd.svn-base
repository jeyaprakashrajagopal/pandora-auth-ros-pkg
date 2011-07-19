
(cl:in-package :asdf)

(defsystem "gui_communications-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "targetPosition" :depends-on ("_package_targetPosition"))
    (:file "_package_targetPosition" :depends-on ("_package"))
  ))