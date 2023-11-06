#!/usr/bin/python3
import rospy
from std_msgs.msg import String

def publisher():
    # 노드 초기화
    rospy.init_node('example_publisher', anonymous=True)

    # 발행자 생성
    pub = rospy.Publisher('example_topic', String, queue_size=10)

    # 루프 속도 설정
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        # 발행할 메시지 생성
        message = "Hello, ROS!"

        # 메시지를 발행
        pub.publish(message)

        # 로그 출력
        rospy.loginfo("Published: %s", message)

        # 루프 속도 맞춤
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass