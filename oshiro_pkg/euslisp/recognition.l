#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")
(hrp2w-init)
(objects *hrp2w*)

(ros::roseus-add-msgs "jsk_pcl_ros")
(ros::roseus "my_recognition")

(setq *tfl* (instance ros::transform-listener :init))

(setq *torso-co* (send (car (send *hrp2w* :torso)) :copy-worldcoords))
;; トルソリンクの一番最初の世界座標系での座標をコピー

(send *tfl* :wait-for-transform
      "CHEST_LINK0" "/camera_remote/depth_registered/cluster_decomposeroutput00" (ros::time 0) 3))
(setq *target-co*
      (send *tfl* :lookup-transform
	    "CHEST_LINK0" "/camera_remote/depth_registered/cluster_decomposeroutput00" (ros::time 0)))
;; TFで認識されたものの一番最初の場所を，CHEST_LINK0から見た相対座標を持ってくる．

(setq *ik-target* (send *torso-co* :transform *target-co*))
;; トルソから計算し直すことでeus世界の世界座標系に変換
(setq *rarm-end* (send *hrp2w* :rarm :end-coords :copy-worldcoords))
;; 右手の先の角度を動かしたくないので，一度今の角度を保持するため座標ごとコピー
(send *rarm-end* :locate (send *ik-target* :pos) :world)
;; コピーした右手座標の位置だけを認識したものにすげ替える
(send *hrp2w* :rarm :inverse-kinematics *rarm-end*
      :rotation-axis nil :debug-view t :torso t
      :link-list (send *hrp2w* :link-list (send *hrp2w* :rarm :end-coords :parent)
		       (car (send *hrp2w* :torso :links))))
;; 腰を使ったIKの使い方．回転角度は無視して位置だけ見る．計算途中の結果を画面に出力する．トルソを使う．IKに使わせるリンクをすべて渡す．
