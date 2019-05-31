1. python==2.7  |  tensorflow==1.5.0  |  cuda9.0  | cudnn7.0.5 에서 확인했습니다.



2. detector.py
line 137 이 detector 에 image를 넣으면 box를 output으로 받는 부분입니다.



3. pip 으로 설치할 수 있는 library가 아닌, cython_bbox 등으로 오류가 난다면, build를 한번 해주면 됩니다.
cd ${ROOT}/lib
bash make.sh

https://github.com/zengarden/light_head_rcnn   의 2번 compiling 부분(+하단 FAQ 부분)

(cf) 원래의 code 는 python3 용으로 되어 있어서, python2 용으로 수정해서 사용했습니다.
