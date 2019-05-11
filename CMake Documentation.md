# CMake 
Considering we have a cpp file/or multiple cpp files. To execute the file we can  write command in terminal to compile in following syntax:<br />
```g++ -o random_executable_file_name name_of_file_to_compliled_and_run.cpp```

But in real world scenerio, it is difficult to mention the file name in above syntax , when we have multiple cpp files related to a particular project and interrelated to each other.<br />
So, it is difficult to mention the name of files in a single terminal command.<br />
To solve such problems, we can use CMake.
 - First create a directory of any name,and move the cpp files in that directory.
 - Now we need to create a file named CMakeLists.txt . This will tell CMake the steps needed to create 'make' file. This 'make' file is used to compile cpp files.
 
 Content of CMakeLists.txt file:<br />
- This will tell CMake the minimum version to use to create make file.
    - ```cmake_minimun_required(VERSION 3.0) ```
- This function called 'set' will allow us to set variables and its corresponding values. For example this variable 'CMAKE_BUILD_TYPE' whose value is 'Debug'. Another example is to set example 'CMAKE_CXX_FLAGS' whose value is '-std=C++14'. This will tell the compiler that the code we are compiling is using C++ 14 standard syntax.   
    - ```set(CMAKE_BUILD_TYPE Debug)```
    - ```set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")```
- After setting variables in CMakeLists file we can tell CMake that we are working on a project by calling the 'project' command.
    - ```project(name_of_project)```
   
- To create the executable or to tell the CMake to create a 'make' file that will create an executable file for our project , we call this command add executable which first contain **the name of the executable we would like to produce for our project**, followed by the necessary source files that 'make' file look for to compile this program.
    - ``` 
      add_executable(
      name_of_project_directory
      source1.cpp
      source2.cpp
      source3.cpp
      ) 
      ```

 - Now after this we can create our 'make' file by ```$ cmake .``` command in the directory of the project. This will create bunch of file , but our main focus is on that 'make' file which will create an executable file for project.
  - ```$ make```<br />
  We can use the following command to run project
  - ```$ ./project_name```
  
   
