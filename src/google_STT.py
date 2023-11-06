import logging
from speech_recognition import Recognizer, Microphone
import speech_recognition
import openai
import os

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)-7s : %(message)s\n')

class Google_STT:
    def __init__(self, lang="ko-KR"):
        self.r = Recognizer()
        self.mic = Microphone(device_index=1)
        self.trigger_words = ["이루", "루머", "헬로", "Hello", "안녕"]
        self.lang = lang
        self.audio = None

    def setup_mic(self):
        with self.mic as source:
            print("주변 소음을 측정합니다.")
            self.r.adjust_for_ambient_noise(source)
            print("소음 측정 완료.")

    def listen(self,isWakeup=False): # 박정현 추가함 (led컨트롤을 위해)
        try:
            with self.mic as source:
                print("음성 수집중")
                if isWakeup :
                    self.audio = self.r.listen(source, timeout=6, phrase_time_limit=2)
                else:
                    self.audio = self.r.listen(source, timeout=6, phrase_time_limit=5)


        except Exception as e:
            logging.error("Error: {}".format(e))
            return 0


    def listen_for_trigger(self):
        try:
            print("[호출명령어] 판단하는 중")
            text = self.r.recognize_google(self.audio, language=self.lang)
            print("[수집된 음성]: {}".format(text))
            return any(trigger_word in text for trigger_word in self.trigger_words)
        except KeyboardInterrupt:
            print(1)
        except speech_recognition.UnknownValueError:
            print("호출 명령어가 없습니다.")
            return 0
        
        except Exception as e:
            logging.error("Error: {}".format(e))
            return 0
        
        

    def listen_for_task(self):
        try:
            print("[Task] 판단하는 중")
            task_text = self.r.recognize_google(self.audio, language=self.lang)
            print("[수집된 Task 내용]: {}".format(task_text))
            return task_text
        
        except speech_recognition.UnknownValueError:
            print("음성 결과를 얻지 못했습니다.")
            return None
        
        except Exception as e:
            logging.error("Error: {}".format(e))
            return False
        except KeyboardInterrupt:
            print(1)



    def run(self):
        while True:
            if self.listen_for_trigger():
                print("호출 명령어가 인식되었습니다.")
                task_text = self.listen_for_task()
                if task_text:
                    return task_text
                else:
                    continue
            else:
                continue



class STT_Agent(Google_STT):
    def __init__(self, lang="ko-KR"):
        super().__init__(lang)
        openai.api_key = os.environ.get('OPENAI_API_KEY') #########################################
        self.destination_list = ['미래관', '정문', '후문', '본관', '학생회관', '시대융합관', '창공관']
        self.PROMPT_FOR_SYSTEM = f"당신은 강아지로, 사용자는 당시에게 여러 동작을 시킬 것입니다.\
                명령을 듣고 대답을 생성하는게 아닌 명령을 분류만 해주면 됩니다. \
                다음 명령어 리스트들에서 명령이 있으면 각 번호로 대답해주세요.\
                if 빙글빙글 돌아 라는 식의 명령을 하면 '1' 이라고 대답.\
                else if 앉으라는 식의 명령을 들어오면 '2' 이라고만 대답.\
                else if 엎드리라는 식의 명령이 들어오면 '3' 이라고만 대답.\
                else if 점프하라거나 뛰라고 명령이 들어오면 '4' 이라고만 대답.\
                else if 춤을 추거나 재롱을 부리라는 식의 의도로 명령하면 '5' 라고만 대답.\
                else if 스트레칭하라고 명령이 들어오면 '6' 이라고만 대답.\
                위 명령어가 인식되면 각 번호를 출력하고, 위 명령어로 분류되지 않으면 '0'이라고 대답하세요.\
                10초안에 대답 못하면 '7'이라고 대답하세요."

        
    def process_user_input(self, messages, model="gpt-3.5-turbo", temp=0):
        response = openai.ChatCompletion.create(
            model=model,
            temperature=temp,
            messages=messages
            )['choices'][0]['message']['content']
        return response

    def filtering_task(self, predicted_text):
        messages = [
            {"role": "system", "content": self.PROMPT_FOR_SYSTEM},
            {"role": "user", "content": "돌아줘"},
            {"role": "assistant", "content": "1"},
            {"role": "user", "content": "앉아"},
            {"role": "assistant", "content": "2"},
            {"role": "user", "content": "엎드려"},
            {"role": "assistant", "content": "3"},
            {"role": "user", "content": "점프"},
            {"role": "assistant", "content": "4"},
            {"role": "user", "content": "dance"},
            {"role": "assistant", "content": "5"},
            {"role": "user", "content": "스트레칭해줘"},
            {"role": "assistant", "content": "6"},
            {"role": "user", "content": predicted_text},
        ]
        filtered_task = self.process_user_input(messages)
        return filtered_task


if __name__ == "__main__":

    stt = STT_Agent()
    stt.setup_mic()
    task_text = stt.run()
    if task_text:
        filtered_task = stt.filtering_task(task_text)
        print("[필터링된 결과값]: {}".format(filtered_task))


