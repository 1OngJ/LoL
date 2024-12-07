
(cl:in-package :asdf)

(defsystem "wall_seg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "WallInfo" :depends-on ("_package_WallInfo"))
    (:file "_package_WallInfo" :depends-on ("_package"))
  ))