#!/usr/bin/env roseus

(ros::roseus "umbrella-catch")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")
(hrp2w-init)
(setq *umb-pos* (make-coords))
(send *umb-pos* :translate (float-vector 800 0 700))

(objects (list *hrp2w* *umb-pos*))
(defun reset ()
  (send *hrp2w* :reset-pose)
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
)

(defun ready ()
  (setq *dest* (send *umb-pos* :copy-worldcoords))
  (send *dest* :translate (float-vector -100 0 0))
  (send *hrp2w* :rarm :inverse-kinematics *dest* :debug-view t :link-list (send *hrp2w* :link-list (send *hrp2w* :rarm :end-coords :parent)))
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (setq *dest* (send *umb-pos* :copy-worldcoords))
  (send *hrp2w* :rarm :inverse-kinematics *dest* :debug-view t :link-list (send *hrp2w* :link-list (send *hrp2w* :rarm :end-coords :parent)))
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
)

(defun up ()
  (setq *dest* (send *umb-pos* :copy-worldcoords))
  (send *dest* :translate (float-vector 0 0 250))
  (send *hrp2w* :rarm :inverse-kinematics *dest* :debug-view t :link-list (send *hrp2w* :link-list (send *hrp2w* :rarm :end-coords :parent)))
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  
  (setq *dest* (send *umb-pos* :copy-worldcoords))
  (send *dest* :translate (float-vector 0 0 500))
  (send *hrp2w* :rarm :inverse-kinematics *dest* :debug-view t :link-list (send *hrp2w* :link-list (send *hrp2w* :rarm :end-coords :parent)))
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
)

(defun back ()
  (setq *dest* (send *umb-pos* :copy-worldcoords))
  (send *dest* :translate (float-vector -200 0 500))
  (send *hrp2w* :rarm :inverse-kinematics *dest* :debug-view t :link-list (send *hrp2w* :link-list (send *hrp2w* :rarm :end-coords :parent)))
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  
  (send *hrp2w* :reset-pose)
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
)