
(cl:in-package :asdf)

(defsystem "tcp2ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "cmd" :depends-on ("_package_cmd"))
    (:file "_package_cmd" :depends-on ("_package"))
    (:file "reach" :depends-on ("_package_reach"))
    (:file "_package_reach" :depends-on ("_package"))
    (:file "readDataAll" :depends-on ("_package_readDataAll"))
    (:file "_package_readDataAll" :depends-on ("_package"))
    (:file "rtkGPSmessage" :depends-on ("_package_rtkGPSmessage"))
    (:file "_package_rtkGPSmessage" :depends-on ("_package"))
  ))