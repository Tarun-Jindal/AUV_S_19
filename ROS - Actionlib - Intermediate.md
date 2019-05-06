# Actionlib - Intermediate
Considering you have already created catkin workspace and actionlib_tutorials named catkin package.Make the catkin workspace and source it to add the workspace to ROS environment.<br/>
Add the following data to actionlib_tutorials/action/Averaging.action
```
#goal definition
int32 samples
---
#result definition
float32 mean
float32 std_dev
---
#feedback
int32 sample
float32 data
float32 mean
float32 std_dev
```
#### Creating Action Messages
To automatically generate the message files during the make process, add the following to CMakeList.txt : 
```
find_package(catkin REQUIRED COMPONENTS actionlib std_msgs message_generation) 
add_action_files(DIRECTORY action FILES Averaging.action)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)
```
Now run : ``` catkin_make ```
### Create the action server
#### 1.Action Server Code:
First, create actionlib_tutorials/src/averaging_server.cpp and add the following code from website:<br/>
[Code link](https://github.com/ros/common_tutorials/blob/hydro-devel/actionlib_tutorials/src/averaging_server.cpp)

#### 2.Compile and Run Action Server
Add the following line to your CMakeLists.txt file:
```
add_executable(averaging_server src/averaging_server.cpp)
target_link_libraries(averaging_server ${catkin_LIBRARIES})
```
Run ```$ roscore```<br/>
and action server ```$ rosrun actionlib_tutorials averaging_server ```<br/>
check running topics and list of topics published and subcribed ``` $ rostopic list -v ```<br/>
also you can check by running rqt graph by following command ``` $ rqt_graph```

### Create the Action Client
First, create actionlib_tutorials/src/averaging_client.cpp and add the following code from website:<br/>
[Code Link](https://github.com/ros/common_tutorials/blob/hydro-devel/actionlib_tutorials/src/averaging_client.cpp)<br/>
Now , Compile and Run :<br/>
Add the following line to your CMakeLists.txt file:
```
add_executable(averaging_client src/averaging_client.cpp)
target_link_libraries(averaging_client ${catkin_LIBRARIES})
```
and now make it execuable by catkin_make.<br/>

Start roscore **if not running already** ```$ roscore```<br/>
and then run client ```$ rosrun actionlib_tutorials averaging_client```<br/>
check if client is running properly , list topics being published or subscribed:``` $ rostopiclist -v ```<br/>
and check rqt graph ```$ rqt_graph```<br/>
sample output if only client is running(**and server is not running**) :<br/>
![](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient%28Threaded%29?action=AttachFile&do=get&target=averaging_action_client.png)

### Running client and server
#### 1.Data Node
Before running the action server and client a data node needs to be created. Create actionlib_tutorials/scripts/gen_numbers.py and place the following inside it: <br/>
[Code Link](https://github.com/ros/common_tutorials/blob/hydro-devel/actionlib_tutorials/scripts/gen_numbers.py)<br/>
the above code generates random numbers with a normal distribution centered around 5 with a standard deviation 1 and publishes the numbers on the /random_number topic.

***Don't forget to make the node executable***
```$ chmod +x gen_numbers.py```
#### 2.Start Data Node
```$ roscore ```<br/>
start data node in new terminal ``` $ rosrun actionlib_tutorials gen_numbers.py```<br/>
you will see : ```generating random numbers```

### Finally Run the whole code
***Basic syntax for running commands***<br/>
```$ rosrun [package_name] [node_name]```<br/>
```$ rostopic echo [topic]```<br/>
```$ rostopic list /topic```<br/>
```$ rostopic list -v```<br/>
Run roscore(**if already not running**) ```$ roscore```<br/>
Run averaging_feedback ```$ rostopic echo /averaging/feedback```<br/>
Run averaging_result ```$ rostopic echo /averaging/result```<br/>
Run averaging_status ```$ rostopic echo /averaging/status```<br/>
Run rqt graph ```$ rqt_graph```<br/>
Sample output:
![](http://wiki.ros.org/actionlib_tutorials/Tutorials/RunningServerAndClientWithNodes?action=AttachFile&do=get&target=averaging_client_server.png)
Run client ```$ rosrun actionlib_tutorials averaging_clieint```<br/>
Run server ```$ rosrun actionlib_tutorials averaging_server```<br/>
*refresh rqt graph to update it to final graph*



