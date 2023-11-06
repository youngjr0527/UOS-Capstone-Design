#!/usr/bin/python3
import argparse
import logging
from threading import Thread
from naver_TTS import TTS_Agent
from google_STT import STT_Agent
from Integrate_LangChain import CampusGuideBot
from bringMenu import UOSMenuScraper
from bringNotice import UOSNoticeScraper
from dotenv import load_dotenv
load_dotenv()

#######################
import rospy
from std_msgs.msg import Int64
from std_msgs.msg import Bool
from std_msgs.msg import String
#######################


logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)-7s : %(message)s\n')

parser = argparse.ArgumentParser()
parser.add_argument('--update', type=bool, default=False, help='Update LLM with Chroma DataBase')
args = parser.parse_args()
update_LLM = False
pub_LED = None

waitForAction = False

class run():
    def __init__(self):
        self.stt = STT_Agent()
        self.bot = CampusGuideBot()
        self.tts = TTS_Agent()

        self.command = Int64()

        
        if args.update or update_LLM:
            rospy.info("Updating LLM...")
            menu_scraper = UOSMenuScraper(save_dir='UOS_DB')
            notice_scraper = UOSNoticeScraper(save_dir='UOS_DB')
            menu_scraper.run()
            notice_scraper.run()
            self.bot.ingest_documents()
            logging.info("LLM updated!")
            
        self.stt.setup_mic()



        

    def run(self):
        global waitForAction, pub_LED
        # TODO: Event 발생시 정적 Action 실행하는 코드 추가. 이것도 Thread로 해서 동시에 실행하면 될듯  
        pub_LED.publish('white')
        self.stt.listen(True)

        pub_LED.publish('black')
        if  self.stt.listen_for_trigger():
            rospy.loginfo("호출 명령어가 인식되었습니다.")


            pub_LED.publish('blue')
            self.stt.listen()
            pub_LED.publish('black')
            # self.tts.generate_audio_and_play("네, 무엇을 도와드릴까요?")
            task_text = self.stt.listen_for_task()
            


            if task_text:
                print('@@@@@@@@@@@@@@@@@', task_text)
                task_text = self.stt.filtering_task(task_text)
                print('#################', task_text)
                if task_text is None:
                    rospy.loginfo("task_text is none")
                    
                elif task_text.startswith("0"):
                    rospy.loginfo(f"[출력된 text]: 0 fail~~~~~~~~~~~~`")

                elif task_text.startswith("1"):
                    rospy.loginfo("1 turn")
                    self.command.data=1
                    pub.publish(self.command)
                    waitForAction = True

                elif task_text.startswith("2"):
                    rospy.loginfo("2 sit down")
                    self.command.data=2
                    pub.publish(self.command)
                    waitForAction = True

                
                elif task_text.startswith("3"):
                    rospy.loginfo("3 lay down")
                    self.command.data=3
                    pub.publish(self.command)
                    waitForAction = True

                elif task_text.startswith("4"):
                    rospy.loginfo("4 jump")
                    self.command.data=4
                    pub.publish(self.command)
                    waitForAction = True

                elif task_text.startswith("5"):
                    rospy.loginfo("5 dance")
                    self.command.data=5
                    pub.publish(self.command)
                    waitForAction = True

                elif task_text.startswith("6"):
                    rospy.loginfo("6 stretching")
                    self.command.data=6
                    pub.publish(self.command)
                    waitForAction = True

                elif task_text.startswith("7"):
                    rospy.loginfo(f"[출력된 text]: 7 long~~~~~~~~~~~~`")


                else:
                    logging.error("### Unexpected input: {}".format(task_text))
                    # break
                    exit()

                if waitForAction:
                    pub_LED.publish('green')

                
            else:
                print("task text is none")

        else:
            pass



def ResultCallback(msg):
    global waitForAction
    waitForAction = False
    print("Action Complete")

# main 함수
if __name__ == "__main__":
    rospy.init_node('langchain', anonymous=True)
    pub = rospy.Publisher('telentCommand', Int64, queue_size=10)
    pub_LED = rospy.Publisher('ledColor',String, queue_size=1)
    subResult = rospy.Subscriber('telentResult', Bool, ResultCallback)
    start = run()

    r = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            if not waitForAction:
                start.run()
            r.sleep()
            
    
    # except rospy.ROSInterruptException:
    #     exit()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted. Shutting down.")

    except KeyboardInterrupt:
        rospy.loginfo(1)
    # except Exception as e:
    #     rospy.loginfo("Error: {}".format(e))
    