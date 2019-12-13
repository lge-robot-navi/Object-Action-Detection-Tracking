### 서울대학교 3차년도 개발 통합 알고리즘 (v2.0)

---
- **서울대학교 인지지능연구실**
#####(**[Perception and Intelligence Laboratory](http://pil.snu.ac.kr/index.do)**)
  - 이규왕
    - 다중객체추적 모듈 코드 작성
  - 엄대호
    - 행동분류 모듈 코드 작성


- **서울대학교 컴퓨터지능 및 패턴인식 연구실**
#####(**[Machine Intelligence and Pattern Recognition Laboratory](http://mipal.snu.ac.kr/index.php/Main_Page)**)
  - 유재영
    - 객체검출 모듈 코드 작성
  - 이호준
    - 객체검출 모듈 코드 작성
  - 정인섭
    - 객체검출 모듈 코드 작성
---

#### 실행 방법 (how to run our code)
- 주간 모듈 ([run_snu_module_iitp_third_year_demo_day.py](src/snu_module/scripts/run_snu_module_iitp_third_year_demo_day.py))
1. roscore 실행
2. bag파일 혹은 realsense 모듈 등, rostopic을 생성해주는 모듈 실행
3. 적절한 rostopic에 대하여 위의 파이썬 파일을 실행

- 야간 모듈 ([run_snu_module_iitp_third_year_demo_night.py](src/snu_module/scripts/run_snu_module_iitp_third_year_demo_night.py))
1. <[DATA](src/snu_module/DATA)> 경로에 이미지 시퀀스 저장
    - 현재 repository에는 샘플의 형태로 저장되어 있음
2. 포맷에 맞게 저장하였다면, 위의 야간 모듈 코드 실행

---
#### 환경 설정 (environments)
- python==2.7

- pyTorch==1.0.0
  * torchVision==0.2.0
  
- CUDA==10.0 (>=9.0)
  * cuDNN==7.5.0
  
- ROS-kinetics
  * rospkg is required in the virtualenv package
  
- opencv-python

- empy

- above packages are in the python2.7 virtualenv

- other dependencies
  * numpy
  * numba
  * scipy
  * FilterPy
  * yaml
  * sklearn
  


