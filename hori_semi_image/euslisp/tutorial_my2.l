#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")
(hrp2w-init)
(objects *hrp2w*)


(ros::roseus "tutorial_desuyo")

(setq *vel_msg* (instance geometry_msgs::Twist :init))



(defun cmd_vel (x y theta)
  (send *vel_msg* :linear :x x)
  (send *vel_msg* :linear :y y)
  (send *vel_msg* :angular :z theta)
  (ros::publish "/cmd_vel" *vel_msg*)
  )

(defun start-vmax ()
  (cmd_vel 0.05 0.0 0.0)
  (unix::usleep (* 1000 1000))
  (cmd_vel 0.0 0.0 0.0)
  )


(defun joy-callback (msg)
  (format t "axes: ~A~%" (send msg :axes))
  (format t "buttons: ~A~%" (send msg :buttons))
  (setq x  (/ (elt (send msg :axes) 1) 10))
  (setq th (/ (elt (send msg :axes) 2) 10))
  (if (= 1 (* (elt (send msg :buttons) 11) (elt (send msg :buttons) 15))) (myreset))
  (if (= 1 (* (elt (send msg :buttons) 11) (elt (send msg :buttons) 13))) (byebye))
  (cmd_vel (* x (elt (send msg :buttons) 10)) 0 (* th (elt (send msg :buttons) 10)))
  )

(defun byebye()

 (send *hrp2w* :rarm :shoulder-p :joint-angle -90)
 (send *hrp2w* :rarm :wrist-y :joint-angle 90)
 (send *hrp2w* :rarm :elbow-p :joint-angle -90)
 (send *ri* :angle-vector (send *hrp2w* :angle-vector) 6000)
 (send *ri* :wait-interpolation)

 (send *hrp2w* :rarm :shoulder-y :joint-angle 20)
 (send *ri* :angle-vector (send *hrp2w* :angle-vector) 1000)
 (send *ri* :wait-interpolation)

 (send *hrp2w* :rarm :shoulder-y :joint-angle -20)
 (send *ri* :angle-vector (send *hrp2w* :angle-vector) 2000)
 (send *ri* :wait-interpolation)

 (send *hrp2w* :rarm :shoulder-y :joint-angle 20)
 (send *ri* :angle-vector (send *hrp2w* :angle-vector) 2000)
 (send *ri* :wait-interpolation)

 (send *hrp2w* :rarm :shoulder-y :joint-angle -20)
 (send *ri* :angle-vector (send *hrp2w* :angle-vector) 2000)
 (send *ri* :wait-interpolation)

 (send *hrp2w* :rarm :shoulder-y :joint-angle 20)
 (send *ri* :angle-vector (send *hrp2w* :angle-vector) 2000)
 (send *ri* :wait-interpolation)

 (send *hrp2w* :rarm :shoulder-y :joint-angle 0)
 (send *ri* :angle-vector (send *hrp2w* :angle-vector) 1000)
 (send *ri* :wait-interpolation)

)

(defun myreset()
  (send *hrp2w* :reset-pose)
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
)



(ros::subscribe "/joy" sensor_msgs::Joy #'joy-callback)
(ros::advertise "/cmd_vel" geometry_msgs::Twist 1)

(do-until-key
 (ros::spin-once)
 )

(cmd_vel 0.0 0.0 0.0)
