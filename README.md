# ABB IRB120 Coppeliasim Moveit Simulation
<p align="center">
  <img  src="https://user-images.githubusercontent.com/81301684/194425810-b7c92c09-1e9a-4b5c-883e-c3838d898cdb.png">
  <img src="https://user-images.githubusercontent.com/81301684/194426104-63a72551-cd23-4683-a02b-cdc791823ae4.png">
  <img width="280" height="300" src="https://user-images.githubusercontent.com/81301684/194424282-e862cb8b-4492-43c2-9e7e-9fe71474a4f0.png">
</p>

# Instructions
## Porpose of simulation
* The porpose of simulation is to test our algorithms for our project **E-waste Non destructive disassembly** 
* there are screw driver at the endeffector and a screw in threaded box that use to test algorithim for lossen the screw

## Steps to run the simulation
When running the simulation theres a sequance you must follow</br>
1. install the dependencies 
```
rosdep install --from-paths src --ignore-src -r -y
```
2. Run ```roscore```
3. open coppeliasim 
```
cd ABB-IRB120-Coppeliasim-Moveit-Simulation
cd src/CoppeliaSim/
./coppeliaSim
```
* then open the scene you want at path ABB-IRB120-Coppeliasim-Moveit-Simulation/src/abb_irb_sim_v2/scene</br>
  there 2 scene : 
  * scene without camera (for controlling purpose)
  * scene with depth and RGB camera
    * open --> open scene ---> which scene
3. launch the sync and connector between moveit and coppeliasim after playing the simulator or press the play icon ▶️
```
cd ABB-IRB120-Coppeliasim-Moveit-Simulation
source devel/setup.bash
roslaunch sim_environment LaunchConnector.launch
```
# ⚠️ Warning
**If you stopped the simulator in any case you must close moveit tool (step4) and close the sync and connector**
**after paly the simulator then re-open them again (repeat **step 3**)**
# Robot Photo
## All robot 
<p align="center">
  <img src="https://user-images.githubusercontent.com/81301684/196560694-a1b7d548-5b70-44ff-8ce3-97238b73f9ff.png">
</p>

## Endeffector
<p align="center">
  <img src="https://user-images.githubusercontent.com/81301684/196561177-c649574f-79f5-4a16-b13d-6e8bfab13142.png">
</p>

# Simulation video
[coppelia_Moveit_sync.webm](https://user-images.githubusercontent.com/81301684/196551505-59a329c2-e061-4812-9c18-5a2c26056d37.webm)
