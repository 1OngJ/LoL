;; Auto-generated. Do not edit!


(when (boundp 'wall_seg::WallInfo)
  (if (not (find-package "WALL_SEG"))
    (make-package "WALL_SEG"))
  (shadow 'WallInfo (find-package "WALL_SEG")))
(unless (find-package "WALL_SEG::WALLINFO")
  (make-package "WALL_SEG::WALLINFO"))

(in-package "ROS")
;;//! \htmlinclude WallInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass wall_seg::WallInfo
  :super ros::object
  :slots (_header _wall_id _corner_position _angle _centroid _normal _min _max ))

(defmethod wall_seg::WallInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:wall_id __wall_id) 0)
    ((:corner_position __corner_position) (instance geometry_msgs::Point :init))
    ((:angle __angle) 0.0)
    ((:centroid __centroid) (instance geometry_msgs::Point :init))
    ((:normal __normal) (instance geometry_msgs::Vector3 :init))
    ((:min __min) (instance geometry_msgs::Point :init))
    ((:max __max) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _wall_id (round __wall_id))
   (setq _corner_position __corner_position)
   (setq _angle (float __angle))
   (setq _centroid __centroid)
   (setq _normal __normal)
   (setq _min __min)
   (setq _max __max)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:wall_id
   (&optional __wall_id)
   (if __wall_id (setq _wall_id __wall_id)) _wall_id)
  (:corner_position
   (&rest __corner_position)
   (if (keywordp (car __corner_position))
       (send* _corner_position __corner_position)
     (progn
       (if __corner_position (setq _corner_position (car __corner_position)))
       _corner_position)))
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:centroid
   (&rest __centroid)
   (if (keywordp (car __centroid))
       (send* _centroid __centroid)
     (progn
       (if __centroid (setq _centroid (car __centroid)))
       _centroid)))
  (:normal
   (&rest __normal)
   (if (keywordp (car __normal))
       (send* _normal __normal)
     (progn
       (if __normal (setq _normal (car __normal)))
       _normal)))
  (:min
   (&rest __min)
   (if (keywordp (car __min))
       (send* _min __min)
     (progn
       (if __min (setq _min (car __min)))
       _min)))
  (:max
   (&rest __max)
   (if (keywordp (car __max))
       (send* _max __max)
     (progn
       (if __max (setq _max (car __max)))
       _max)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _wall_id
    1
    ;; geometry_msgs/Point _corner_position
    (send _corner_position :serialization-length)
    ;; float64 _angle
    8
    ;; geometry_msgs/Point _centroid
    (send _centroid :serialization-length)
    ;; geometry_msgs/Vector3 _normal
    (send _normal :serialization-length)
    ;; geometry_msgs/Point _min
    (send _min :serialization-length)
    ;; geometry_msgs/Point _max
    (send _max :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _wall_id
       (write-byte _wall_id s)
     ;; geometry_msgs/Point _corner_position
       (send _corner_position :serialize s)
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Point _centroid
       (send _centroid :serialize s)
     ;; geometry_msgs/Vector3 _normal
       (send _normal :serialize s)
     ;; geometry_msgs/Point _min
       (send _min :serialize s)
     ;; geometry_msgs/Point _max
       (send _max :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _wall_id
     (setq _wall_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Point _corner_position
     (send _corner_position :deserialize buf ptr-) (incf ptr- (send _corner_position :serialization-length))
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Point _centroid
     (send _centroid :deserialize buf ptr-) (incf ptr- (send _centroid :serialization-length))
   ;; geometry_msgs/Vector3 _normal
     (send _normal :deserialize buf ptr-) (incf ptr- (send _normal :serialization-length))
   ;; geometry_msgs/Point _min
     (send _min :deserialize buf ptr-) (incf ptr- (send _min :serialization-length))
   ;; geometry_msgs/Point _max
     (send _max :deserialize buf ptr-) (incf ptr- (send _max :serialization-length))
   ;;
   self)
  )

(setf (get wall_seg::WallInfo :md5sum-) "94095140c7fde5508eeca8261235bdf7")
(setf (get wall_seg::WallInfo :datatype-) "wall_seg/WallInfo")
(setf (get wall_seg::WallInfo :definition-)
      "std_msgs/Header header
uint8 wall_id
geometry_msgs/Point corner_position
float64 angle
geometry_msgs/Point centroid
geometry_msgs/Vector3 normal
geometry_msgs/Point min
geometry_msgs/Point max
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :wall_seg/WallInfo "94095140c7fde5508eeca8261235bdf7")


