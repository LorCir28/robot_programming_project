;; Auto-generated. Do not edit!


(when (boundp 'my_package::Num)
  (if (not (find-package "MY_PACKAGE"))
    (make-package "MY_PACKAGE"))
  (shadow 'Num (find-package "MY_PACKAGE")))
(unless (find-package "MY_PACKAGE::NUM")
  (make-package "MY_PACKAGE::NUM"))

(in-package "ROS")
;;//! \htmlinclude Num.msg.html


(defclass my_package::Num
  :super ros::object
  :slots (_ranges ))

(defmethod my_package::Num
  (:init
   (&key
    ((:ranges __ranges) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _ranges __ranges)
   self)
  (:ranges
   (&optional __ranges)
   (if __ranges (setq _ranges __ranges)) _ranges)
  (:serialization-length
   ()
   (+
    ;; float32[] _ranges
    (* 4    (length _ranges)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _ranges
     (write-long (length _ranges) s)
     (dotimes (i (length _ranges))
       (sys::poke (elt _ranges i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _ranges
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ranges (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _ranges i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get my_package::Num :md5sum-) "b59fbb52705f63cb3fada08d311c9c4d")
(setf (get my_package::Num :datatype-) "my_package/Num")
(setf (get my_package::Num :definition-)
      "float32[] ranges

")



(provide :my_package/Num "b59fbb52705f63cb3fada08d311c9c4d")


