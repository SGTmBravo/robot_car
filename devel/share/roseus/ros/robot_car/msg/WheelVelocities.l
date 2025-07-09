;; Auto-generated. Do not edit!


(when (boundp 'robot_car::WheelVelocities)
  (if (not (find-package "ROBOT_CAR"))
    (make-package "ROBOT_CAR"))
  (shadow 'WheelVelocities (find-package "ROBOT_CAR")))
(unless (find-package "ROBOT_CAR::WHEELVELOCITIES")
  (make-package "ROBOT_CAR::WHEELVELOCITIES"))

(in-package "ROS")
;;//! \htmlinclude WheelVelocities.msg.html


(defclass robot_car::WheelVelocities
  :super ros::object
  :slots (_front_left _front_right _rear_left _rear_right _duration ))

(defmethod robot_car::WheelVelocities
  (:init
   (&key
    ((:front_left __front_left) 0.0)
    ((:front_right __front_right) 0.0)
    ((:rear_left __rear_left) 0.0)
    ((:rear_right __rear_right) 0.0)
    ((:duration __duration) 0.0)
    )
   (send-super :init)
   (setq _front_left (float __front_left))
   (setq _front_right (float __front_right))
   (setq _rear_left (float __rear_left))
   (setq _rear_right (float __rear_right))
   (setq _duration (float __duration))
   self)
  (:front_left
   (&optional __front_left)
   (if __front_left (setq _front_left __front_left)) _front_left)
  (:front_right
   (&optional __front_right)
   (if __front_right (setq _front_right __front_right)) _front_right)
  (:rear_left
   (&optional __rear_left)
   (if __rear_left (setq _rear_left __rear_left)) _rear_left)
  (:rear_right
   (&optional __rear_right)
   (if __rear_right (setq _rear_right __rear_right)) _rear_right)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; float32 _front_left
    4
    ;; float32 _front_right
    4
    ;; float32 _rear_left
    4
    ;; float32 _rear_right
    4
    ;; float32 _duration
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _front_left
       (sys::poke _front_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _front_right
       (sys::poke _front_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rear_left
       (sys::poke _rear_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rear_right
       (sys::poke _rear_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _front_left
     (setq _front_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _front_right
     (setq _front_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rear_left
     (setq _rear_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rear_right
     (setq _rear_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _duration
     (setq _duration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robot_car::WheelVelocities :md5sum-) "b1a59607ac24381c9b60b103b66fe21a")
(setf (get robot_car::WheelVelocities :datatype-) "robot_car/WheelVelocities")
(setf (get robot_car::WheelVelocities :definition-)
      "float32 front_left
float32 front_right
float32 rear_left
float32 rear_right
float32 duration

")



(provide :robot_car/WheelVelocities "b1a59607ac24381c9b60b103b66fe21a")


