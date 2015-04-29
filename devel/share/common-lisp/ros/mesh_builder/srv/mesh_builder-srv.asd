
(cl:in-package :asdf)

(defsystem "mesh_builder-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :moveit_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "MeshCloud" :depends-on ("_package_MeshCloud"))
    (:file "_package_MeshCloud" :depends-on ("_package"))
  ))