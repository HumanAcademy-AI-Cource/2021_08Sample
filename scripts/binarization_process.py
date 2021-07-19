#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 必要なライブラリをインポート
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
 

def image_callback(input_image):
    """
    グレースケール画像を二値化画像に変換する関数
    """

    # 画像をROSのデータ形式からOpenCV形式に変換
    bridge = CvBridge()
    img_gray = bridge.imgmsg_to_cv2(input_image, "mono8")

    # グレースケール画像を二値化画像に変換
    # 閾値は0~255で設定可能, 閾値より小さい画素は黒, 閾値より大きい画素は白に変換
    # 今回は閾値を100に設定
    ret_th, img_th = cv2.threshold(img_gray, 100, 255, cv2.THRESH_BINARY)

    # 画像をOpenCV形式からROSのデータ形式に変換
    img_msg = bridge.cv2_to_imgmsg(img_th, "mono8")

    # パブリッシャを定義
    pub = rospy.Publisher('image_threshold', Image, queue_size=10)
    
    # 2値化画像をパブリッシュ
    pub.publish(img_msg)
    
if __name__ == '__main__':
    
    # ノードを宣言
    rospy.init_node('binarization_process')
    # サブスクライバを定義
    # 画像をサブスクライブするとき、image_callback関数を呼び出す
    rospy.Subscriber("/image_gray", Image, image_callback)
    # プロセス終了までループ
    rospy.spin()