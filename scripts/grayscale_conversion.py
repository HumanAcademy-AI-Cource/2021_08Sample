#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 必要なライブラリをインポート
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def image_callback(input_image):
    """
    受け取った画像をグレースケールに変換する関数
    """

    # 画像をROSのデータ形式からOpenCV形式に変換
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(input_image, "bgr8")

    # 画像をグレースケールに変換
    # グレースケール画像: 1画素を白黒の濃淡で表した画像, 濃淡の度合いは0~255の数値で表現
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 画像をOpenCV形式からROSのデータ形式に変換
    img_msg = bridge.cv2_to_imgmsg(img_gray, "mono8")

    # パブリッシャを定義
    pub = rospy.Publisher('image_gray', Image, queue_size=10)

    # グレースケール画像をパブリッシュ
    pub.publish(img_msg)
    
if __name__ == '__main__':
    
    # ノードを宣言
    rospy.init_node('grayscale_conversion')
    # サブスクライバを定義
    # 画像をサブスクライブするとき、image_callback関数を呼び出す
    rospy.Subscriber("/image_raw", Image, image_callback)
    # プロセス終了までループ
    rospy.spin()