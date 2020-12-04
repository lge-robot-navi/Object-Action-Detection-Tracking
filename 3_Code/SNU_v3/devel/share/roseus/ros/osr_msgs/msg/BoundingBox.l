;; Auto-generated. Do not edit!


(when (boundp 'osr_msgs::BoundingBox)
  (if (not (find-package "OSR_MSGS"))
    (make-package "OSR_MSGS"))
  (shadow 'BoundingBox (find-package "OSR_MSGS")))
(unless (find-package "OSR_MSGS::BOUNDINGBOX")
  (make-package "OSR_MSGS::BOUNDINGBOX"))

(in-package "ROS")
;;//! \htmlinclude BoundingBox.msg.html


(defclass osr_msgs::BoundingBox
  :super ros::object
  :slots (_x _y _height _width ))

(defmethod osr_msgs::BoundingBox
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    ((:height __height) 0)
    ((:width __width) 0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _height (round __height))
   (setq _width (round __width))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:serialization-length
   ()
   (+
    ;; uint32 _x
    4
    ;; uint32 _y
    4
    ;; uint32 _height
    4
    ;; uint32 _width
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _x
       (write-long _x s)
     ;; uint32 _y
       (write-long _y s)
     ;; uint32 _height
       (write-long _height s)
     ;; uint32 _width
       (write-long _width s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get osr_msgs::BoundingBox :md5sum-) "7c3a991d96caa1d962ef7cdfbc63f853")
(setf (get osr_msgs::BoundingBox :datatype-) "osr_msgs/BoundingBox")
(setf (get osr_msgs::BoundingBox :definition-)
      "####################
# osr_msgs bounding box message
####################

uint32 x
uint32 y 
uint32 height
uint32 width

")



(provide :osr_msgs/BoundingBox "7c3a991d96caa1d962ef7cdfbc63f853")


