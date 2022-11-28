# mpc_casadi
맞다 동규 push 했는데 이거 decompROS도 깔아야 sfc 돌아간디
https://github.com/sikang/DecompROS

# dynamic object environment setting

```
$ cd ~/simulation_setup
$ mkdir build && cd build
$ cmake ..
$ make

$ cd ~/simulation_setup
$ echo "export GAZEBO_MODEL_PATH=$(pwd):$GAZEBO_MODEL_PATH" >> ~/.bashrc
$ source ~/.bashrc

$ cd ~/simulation_setup/build
$ echo "export GAZEBO_PLUGIN_PATH=$(pwd):$GAZEBO_PLUGIN_PATH" >> ~/.bashrc
$ source ~/.bashrc
```

## How to use

animated_box.cc 는 플러그인 생성용! 여러개 생성하고, 각 waypoint 설정하면 됨.

CMakeLists.txt에 적절히 추가할 것.

"drc_practice_blue_cyclinder" 는 동적 장애물로 가져온 모델.

model.sdf 안에 다음 tag 추가할 것.

```
        ...
        <static>1</static>
        <allow_auto_disable>1</allow_auto_disable>
        <plugin name='push_animate' filename='libanimated_box.so'/>
    </model>
</sdf>
```

원하는 map.world 들어가서, 아래처럼 모델 불러오면 됨
```
        <include>
            <uri>model://drc_practice_blue_cylinder</uri>      
            <name>blue_cylinder</name>
        </include>
```
여러개 불러올거면, 각 모델 복붙한 다음에 이름 안꼬이게 자알 설정하기.

끄읕!