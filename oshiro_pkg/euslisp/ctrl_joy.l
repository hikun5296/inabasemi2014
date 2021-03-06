#!/usr/bin/env roseus

(ros::roseus "ctrl_joy")
(ros::advertise "/cmd_vel" geometry_msgs::Twist 1)

(setq *vel_msg* (instance geometry_msgs::Twist :init))

(defun cmd_vel (msg)
  (if (= (elt (send msg :buttons) 10) 1)
      (progn
	(send *vel_msg* :linear :x  (* 0.1 (elt (send msg :axes) 1 )))
	(send *vel_msg* :linear :y  (* 0.1 (elt (send msg :axes) 0)))
	(send *vel_msg* :angular :z (* 0.1 (elt (send msg :axes) 2)))
	(ros::publish "/cmd_vel" *vel_msg*)
	)
    (progn
      (send *vel_msg* :linear :x 0)
      (send *vel_msg* :linear :y 0)
      (send *vel_msg* :angular :z 0)
      (ros::publish"/cmd_vel"  *vel_msg*)
      )
    )
  )

(ros::subscribe "/joy" sensor_msgs::Joy #'cmd_vel)

(do-until-key
 (ros::spin-once)
 )

