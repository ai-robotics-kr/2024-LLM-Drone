# Week 3

## LLM 필수 시청 인증\* 영상 (총 시청시간 50분)

- [Open Source LMs](https://www.youtube.com/watch?v=TLisXrictso&list=WL&index=3&t=247s&ab_channel=%EA%B3%A0%EB%A0%A4%EB%8C%80%ED%95%99%EA%B5%90%EC%82%B0%EC%97%85%EA%B2%BD%EC%98%81%EA%B3%B5%ED%95%99%EB%B6%80DSBA%EC%97%B0%EA%B5%AC%EC%8B%A4) (논문제목 : LLaMA: Open and Efficient Foundation Language Models)
- ByteByteGo의 [How ChatGPT Works Technically | ChatGPT Architecture](https://youtu.be/bSvTVREwSNw?si=V5Ydnw7I3Du5YNQ6)
- AI Papers Academy의 [LLM in a flash](https://www.youtube.com/watch?v=nI3uYT3quxQ&t=168s)

> 📜인증 방법

[](http://dsba.korea.ac.kr/seminar/?mod=document&pageid=1&keyword=llama&uid=2738)

1. 위 `dsba 댓글 내용`을 참조하여 3개의 영상에 대해서 최소 100자 이상 최대 3000자 이하의 글을 작성한다.
2. 글은 [](https://www.notion.so/9c96db0aaae74554892e15bfa3a05569?pvs=21) 의 개인 페이지에 작성한다.
3. 글의 양식은 노션, 개인 블로그, SNS, PPT 형식의 발표자료 모두 상관없다.

## 드론 시뮬레이터 환경 세팅

### Window 환경에서

[링크](https://github.com/microsoft/PromptCraft-Robotics/blob/main/chatgpt_airsim/README.md) 따라 해보기 (ChatGPT for Robotics 직접 돌려보기)

### Linux 환경에서

[기본환경 세팅 (ROS, PX4)](https://www.notion.so/ROS-PX4-38b8f813c1fd4e1f86a74978195d92df?pvs=21) 을 따라 해보기

## 스터디 진행내용

### LLM

**김미향님 발표**

[240119_LLM](https://www.notion.so/240119_LLM-011964b5204745619fc8993b4fd3ba92?pvs=21)

**정소라님 발표**

[2024-01-19 3주차 ](https://www.notion.so/2024-01-19-3-120f7376a7544fb0a10ca4f76181af19?pvs=21)

### 드론

- 환경 구성 중 에러 사항 공유
- 다음 주차 진행 상황 논의 ( **우성님 스터디 페이지, 공지사항 참고** )
  - 에러가 발생한 분들 시뮬레이션 환경 세팅 완료 및 에러/질문사항 공유
  - ChatGPT-AirSim 코드를 ros 환경으로 migration (mavros 기반)
  - ROS 혹은 mavros 를 익힐 수 있는 자료나 영상 공유
- 최종적으로 어떤 LLM 드론을 할 것인지?
  - 우성) 드론을 평가하는 기준이 필요함. 응답하는 시간과 안정성, 지시를 이행하는 성공률 등이 있을 수 있다고 교수님과 논의. 이 외에 몇 가지 좋은 기준들 제시.
  - 영화) 초반에는 LLM 과 드론에 대해 학습해보는 것을 얘기하였고, 명확하게 무엇을 할지는 아직 정하지 않은 느낌이 있음.
  - 찬준) 시뮬레이션과 ROS 기반으로 코드를 작성해보고 과정에서 발생하는 문제점들을 제시하고 이를 해결해보는 논의를 주차별로 진행. ex) 프롬프트 엔지니어링, 파인튜닝, 추가적인 모델 활용 혹은 명확한 task 지정
