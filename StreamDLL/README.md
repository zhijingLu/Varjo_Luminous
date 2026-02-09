# VarjoStreamDLL 

This folder is used for making a dll.
If you want to directly get the dll, please download the zip file in DLL folder directly.

## Prepare lib 
1. Please download the Varjo Native Experimental SDK:https://developer.varjo.com/downloads#native-developer-assets
   
   version:4.12.0

   **!!  Here is a problem. experimental is not suggested to use. Now it is used in pointcloud.cpp**
   
   **!!  If just use V02 without any pointcloud info, please use the varjo native sdk in link.**
   
1. Please download the opencv 4.8.0 from [opencv github](https://github.com/opencv/opencv/releases/tag/4.8.0) and opencv-contrib 4.8.0 from [contrib github](https://github.com/opencv/opencv_contrib/tree/4.8.0)  and compile as lib and dll.

## Compile
1. please open x64 Native Tools Command Prompt for VS.
2. open x.bat(build_dll.bat and OpenCVdll.BAT) and change the path in setting area. (if varjo sdk is experimental, please also change the pathname)
4. run :
```
 x.bat 
```
4. You will find the dll in build folder
