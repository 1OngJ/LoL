; Auto-generated. Do not edit!


(cl:in-package wall_seg-msg)


;//! \htmlinclude WallInfo.msg.html

(cl:defclass <WallInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (wall_id
    :reader wall_id
    :initarg :wall_id
    :type cl:fixnum
    :initform 0)
   (corner_position
    :reader corner_position
    :initarg :corner_position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (centroid
    :reader centroid
    :initarg :centroid
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (normal
    :reader normal
    :initarg :normal
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (min
    :reader min
    :initarg :min
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (max
    :reader max
    :initarg :max
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass WallInfo (<WallInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WallInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WallInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wall_seg-msg:<WallInfo> is deprecated: use wall_seg-msg:WallInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wall_seg-msg:header-val is deprecated.  Use wall_seg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'wall_id-val :lambda-list '(m))
(cl:defmethod wall_id-val ((m <WallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wall_seg-msg:wall_id-val is deprecated.  Use wall_seg-msg:wall_id instead.")
  (wall_id m))

(cl:ensure-generic-function 'corner_position-val :lambda-list '(m))
(cl:defmethod corner_position-val ((m <WallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wall_seg-msg:corner_position-val is deprecated.  Use wall_seg-msg:corner_position instead.")
  (corner_position m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <WallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wall_seg-msg:angle-val is deprecated.  Use wall_seg-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'centroid-val :lambda-list '(m))
(cl:defmethod centroid-val ((m <WallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wall_seg-msg:centroid-val is deprecated.  Use wall_seg-msg:centroid instead.")
  (centroid m))

(cl:ensure-generic-function 'normal-val :lambda-list '(m))
(cl:defmethod normal-val ((m <WallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wall_seg-msg:normal-val is deprecated.  Use wall_seg-msg:normal instead.")
  (normal m))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <WallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wall_seg-msg:min-val is deprecated.  Use wall_seg-msg:min instead.")
  (min m))

(cl:ensure-generic-function 'max-val :lambda-list '(m))
(cl:defmethod max-val ((m <WallInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wall_seg-msg:max-val is deprecated.  Use wall_seg-msg:max instead.")
  (max m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WallInfo>) ostream)
  "Serializes a message object of type '<WallInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wall_id)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'corner_position) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'centroid) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'normal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WallInfo>) istream)
  "Deserializes a message object of type '<WallInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wall_id)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'corner_position) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'centroid) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'normal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WallInfo>)))
  "Returns string type for a message object of type '<WallInfo>"
  "wall_seg/WallInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WallInfo)))
  "Returns string type for a message object of type 'WallInfo"
  "wall_seg/WallInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WallInfo>)))
  "Returns md5sum for a message object of type '<WallInfo>"
  "94095140c7fde5508eeca8261235bdf7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WallInfo)))
  "Returns md5sum for a message object of type 'WallInfo"
  "94095140c7fde5508eeca8261235bdf7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WallInfo>)))
  "Returns full string definition for message of type '<WallInfo>"
  (cl:format cl:nil "std_msgs/Header header~%uint8 wall_id~%geometry_msgs/Point corner_position~%float64 angle~%geometry_msgs/Point centroid~%geometry_msgs/Vector3 normal~%geometry_msgs/Point min~%geometry_msgs/Point max~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WallInfo)))
  "Returns full string definition for message of type 'WallInfo"
  (cl:format cl:nil "std_msgs/Header header~%uint8 wall_id~%geometry_msgs/Point corner_position~%float64 angle~%geometry_msgs/Point centroid~%geometry_msgs/Vector3 normal~%geometry_msgs/Point min~%geometry_msgs/Point max~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WallInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'corner_position))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'centroid))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'normal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WallInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'WallInfo
    (cl:cons ':header (header msg))
    (cl:cons ':wall_id (wall_id msg))
    (cl:cons ':corner_position (corner_position msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':centroid (centroid msg))
    (cl:cons ':normal (normal msg))
    (cl:cons ':min (min msg))
    (cl:cons ':max (max msg))
))
