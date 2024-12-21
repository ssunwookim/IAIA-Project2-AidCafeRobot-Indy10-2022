#!/usr/bin/env python3
# -*- coding:utf-8 -*- 

import rospy
from sensor_msgs.msg import Image   # sensor_msgs 패키지로부터 Image 메시지 타입을 import
from cv_bridge import CvBridge, CvBridgeError      # cv_bridge 라이브러리 : OpenCV 이미지와 ROS 메시지 간의 변환 가능
from pyzbar.pyzbar import decode
import requests
from bs4 import BeautifulSoup
from indy_driver.msg import drink_info

class QRClassifierNode:
    def __init__(self):
        # ROS 초기화
        rospy.init_node("qr_classifier", anonymous=True)

        # ROS 메시지 변환 도구 (OpenCV <-> ROS Image)
        self.bridge = CvBridge()

        # 이미지 구독자 설정
        self.image_sub = rospy.Subscriber("camera/image_raw1", Image, self.process_image)

        # 음료 정보 퍼블리셔 설정
        self.drink_pub = rospy.Publisher("drink_info", drink_info, queue_size=10)

        # QR 코드 안정성 타이머 및 URL
        self.timer_start = None
        self.last_recognized_data = None

        # 연속 인식 카운트 및 임계값 설정
        self.chk_cnt = 0
        self.thresh_cnt = 20  # 연속적으로 20개 QR 코드가 인식될 때만 실행

        # 음료 키워드 리스트
        # self.drink_keywords = ["Greengrape", "Peach", "Orange", "Strawberry", "Blueberry"] # Deleted Peach
        self.drink_keywords = ["Greengrape", "Orange", "Strawberry", "Peach"]
    def process_image(self, data):
        try:
            # 수신된 Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # 이미지에서 QR 코드 디코딩
            qr_codes = decode(cv_image)
            if qr_codes:
                for qr_code in qr_codes:
                    # QR 코드 데이터 가져오기
                    qr_data = qr_code.data.decode("utf-8")
                    rospy.loginfo("QR Code Detected: %s", qr_data)

                    # 텍스트인 경우 바로 처리
                    self.extract_and_publish_drink_info_from_text(qr_data)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", str(e))

    def extract_and_publish_drink_info(self, url):
        try:
            # URL에 GET 요청
            response = requests.get(url)
            if response.status_code == 200:
                # 웹 페이지 내용 파싱
                soup = BeautifulSoup(response.text, "html.parser")
                page_text = " ".join(soup.get_text().split())

                # 음료 키워드 탐색
                for keyword in self.drink_keywords:
                    if keyword in page_text:
                        # ROS 메시지 생성 및 퍼블리시
                        drink_message = drink_info()
                        drink_message.name = keyword
                        self.drink_pub.publish(drink_message)
                        rospy.loginfo("Published Drink: %s", keyword)
                        return

                # 키워드가 없는 경우 로그 경고
                rospy.logwarn("No drink keywords found in the webpage.")
            else:
                rospy.logerr("Failed to fetch URL: HTTP %d", response.status_code)
        except Exception as e:
            rospy.logerr("Error fetching or processing URL: %s", str(e))

    def extract_and_publish_drink_info_from_text(self, text):
        # 텍스트에서 음료 키워드 탐색
        for keyword in self.drink_keywords:
            if keyword in text:
                # 연속적인 인식 여부 체크
                if text == self.last_recognized_data:
                    self.chk_cnt += 1
                else:
                    self.chk_cnt = 1  # 처음 인식된 경우

                # 일정 횟수 이상 연속 인식되었을 때만 퍼블리시
                if self.chk_cnt >= self.thresh_cnt:
                    # ROS 메시지 생성 및 퍼블리시
                    drink_message = drink_info()
                    drink_message.name = keyword
                    self.drink_pub.publish(drink_message)
                    rospy.loginfo("Published Drink: %s", keyword)
                    self.chk_cnt = 0  # 카운트 리셋

                self.last_recognized_data = text  # 마지막 인식된 데이터를 갱신
                return

        # 텍스트에서 키워드가 없는 경우 로그 경고
        rospy.logwarn("No drink keywords found in the text.")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        qr_classifier = QRClassifierNode()
        qr_classifier.run()
    except rospy.ROSInterruptException:
        pass
