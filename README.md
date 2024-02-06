# ChatGPT-PX4 Gazebo Interface

## Getting started
As a prerequisite, you need to create an OpenAI account that allows access to ChatGPT through the OpenAI API. You can do so by visiting https://platform.openai.com and signing up for an account.

### Installation & Build
```bash
cd catkin_ws/src
git clone -b chatgpt_gazebo git@github.com:ai-robotics-kr/2024-LLM-Drone.git # SSH
# git clone -b chatgpt_gazebo https://github.com/ai-robotics-kr/2024-LLM-Drone.git # HTTPS
pip install -r requirements.txt
cd .. && catkin build chatgpt_gazebo
```
Set up an API key by visiting https://platform.openai.com/account/api-keys. Copy the API key and paste it in the OPENAI_API_KEY field of config.json.

## How to Run
```bash
# roslaunch chatgpt_gazebo env.launch
# roslaunch chatgpt_gazebo chatgpt_gazebo.launch
```

### TODO
*Updated in Feb 02*
- [ ] : ChatGPT-Airsim 에서는 `exec` 으로 python 함수 실행하는 방식을 ROS 환경에 맞게 수정
- [ ] : PX4 - Gazebo 환경 실행 런치파일