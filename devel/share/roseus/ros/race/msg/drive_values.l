;; Auto-generated. Do not edit!


(when (boundp 'race::drive_values)
  (if (not (find-package "RACE"))
    (make-package "RACE"))
  (shadow 'drive_values (find-package "RACE")))
(unless (find-package "RACE::DRIVE_VALUES")
  (make-package "RACE::DRIVE_VALUES"))

(in-package "ROS")
;;//! \htmlinclude drive_values.msg.html


(defclass race::drive_values
  :super ros::object
  :slots (_throttle _steering ))

(defmethod race::drive_values
  (:init
   (&key
    ((:throttle __throttle) 0)
    ((:steering __steering) 0)
    )
   (send-super :init)
   (setq _throttle (round __throttle))
   (setq _steering (round __steering))
   self)
  (:throttle
   (&optional __throttle)
   (if __throttle (setq _throttle __throttle)) _throttle)
  (:steering
   (&optional __steering)
   (if __steering (setq _steering __steering)) _steering)
  (:serialization-length
   ()
   (+
    ;; int16 _throttle
    2
    ;; int16 _steering
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _throttle
       (write-word _throttle s)
     ;; int16 _steering
       (write-word _steering s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _throttle
     (setq _throttle (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _steering
     (setq _steering (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get race::drive_values :md5sum-) "0194e179a126494f7021e4f23090fd6b")
(setf (get race::drive_values :datatype-) "race/drive_values")
(setf (get race::drive_values :definition-)
      "int16 throttle
int16 steering
")



(provide :race/drive_values "0194e179a126494f7021e4f23090fd6b")


