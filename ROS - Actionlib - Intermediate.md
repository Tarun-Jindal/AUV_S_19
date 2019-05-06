# Actionlib - Intermediate
Considering you have already created catkin workspace and actionlib_tutorials named catkin package.Make the catkin workspace and source it to add the workspace to ROS environment.
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
First, create actionlib_tutorials/src/averaging_server.cpp and add the following code from website:
[Code link](https://github.com/ros/common_tutorials/blob/hydro-devel/actionlib_tutorials/src/averaging_server.cpp)

#### 2.Compile and Run Action Server
Add the following line to your CMakeLists.txt file:
```
add_executable(averaging_server src/averaging_server.cpp)
target_link_libraries(averaging_server ${catkin_LIBRARIES})
```
Run ```$ roscore```
and action server ```$ rosrun actionlib_tutorials averaging_server ```
check running topics and list of topics published and subcribed ``` $ rostopic list -v ```

also you can check by running rqt graph by following command ``` $ rqt_graph``` 

### Create the Action Client
First, create actionlib_tutorials/src/averaging_client.cpp and add the following code from website:
[Code Link](https://github.com/ros/common_tutorials/blob/hydro-devel/actionlib_tutorials/src/averaging_client.cpp)
Now , Compile and Run :
Add the following line to your CMakeLists.txt file:
```
add_executable(averaging_client src/averaging_client.cpp)
target_link_libraries(averaging_client ${catkin_LIBRARIES})
```
and now make it execuable by catkin_make.

Start roscore **if not running already** ```$ roscore```
and then run client ```rosrun actionlib_tutorials averagin_client```

check if client is running properly , list topics being published or subscribed:``` $ rostopiclist -v ```

and check rqt graph ```$ rqt_graph```
sample output if only client is running(**and server is not running**) :
![](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient%28Threaded%29?action=AttachFile&do=get&target=averaging_action_client.png)

### Running client and server
#### 1.Data Node
Before running the action server and client a data node needs to be created. Create actionlib_tutorials/scripts/gen_numbers.py and place the following inside it: 
[Code Link](https://github.com/ros/common_tutorials/blob/hydro-devel/actionlib_tutorials/scripts/gen_numbers.py)
the above code generates random numbers with a normal distribution centered around 5 with a standard deviation 1 and publishes the numbers on the /random_number topic.

***Don't forget to make the node executable***
```$ chmod +x gen_numbers.py```
#### 2.Start Data Node
```$ roscore ```
start data node in new terminal ``` $ rosrun actionlib_tutorials gen_numbers.py```

you will see : ```generating random numbers```

### Finally Run the whole code
***Basic syntax for running commands***
```$ rosrun [package_name] [node_name]```
```$ rostopic echo [topic]```
```$ rostopic list /topic```
```$ rostopic list -v```


Run roscore(**if already not running**) ```$ roscore```
Run averaging_feedback ```$ rostopic echo /averaging/feedback```
Run averaging_result ```$ rostopic echo /averaging/result```
Run averaging_status ```$ rostopic echo /averaging/status```
Run rqt graph ```$ rqt_graph```
Sample output:
![](http://wiki.ros.org/actionlib_tutorials/Tutorials/RunningServerAndClientWithNodes?action=AttachFile&do=get&target=averaging_client_server.png)
Run client ```$ rosrun actionlib_tutorials averaging_clieint```
Run server ```$ rosrun actionlib_tutorials averaging_server```
*refresh rqt graph to update it to final graph*



