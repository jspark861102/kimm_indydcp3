# kimm_phri_panda_husky
KIMM human-robot collaborative transportation application with neuromeka indy7 & agilex ranger Platform

## 1. Prerequisites
```bash
git clone https://github.com/jspark861102/kimm_indydcp3.git -b develop
git clone https://github.com/jspark861102/kimm_robotiq.git #pymodbuss<2.5.3 required
git clone https://github.com/jspark861102/ranger_indy_description.git
git clone https://github.com/jspark861102/ranger_ros.git
git clone https://github.com/jspark861102/ugv_sdk.git
```

## 2. Run
```bash
roslaunch robotiq_2f_gripper_control robotiq_action_server.launch
roslaunch ranger_bringup ranger.launch
roslaunch kimm_indydcp3 indydcp3.launch
```

