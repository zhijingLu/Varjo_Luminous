# Varjo Project
## install
    1. Install Unity Hub : https://public-cdn.cloud.unity3d.com/hub/prod/UnityHubSetup.exe

        Version : 3.15.1
        Make an Unity Account

    2. Install Varjo Base : Create a Varjo Acc (need a commercial account later)

        Install Varjo 3 XR 3 related Varjo base :
        https://account.varjo.com/downloads/varjoheadsets/varjo_base/varjo_base/versions

        Version : 3.7.2

    3. Connect Varjo Hardware to the workstation

        Follow the step/ instruction in Varjo Base. 


    4. Open Unity Hub > go to Projects > add a project from disk > install the necessary unity version. 

        Version : 2021.3.15f1

        A new unity hub window will pop up

    5. Install the Varjo XR plugin (if not in package manager)

        Go to window > package manager > add package using git url 

        version:https://github.com/varjocom/VarjoUnityXRPlugin.git#3.4.0

        Link : Getting Started with Varjo XR Plugin for Unity | Varjo for developers

        opencv version:4.8.0
        
        opencv-contrib: 4.8.0
## Add DLL
1. Please follow the  [c++ readme](c++/README.md) to compile a VarjoStream.dll file
2. Please find the OpenCV-4.8.0 dll.
3. Add dll files into Assert folder.
   
## problem 
    1. if we meet the problem that when we run the text_example, the text is not shown in front of camera, not rotate with camera, or direction of text is not facing the camera directly, please 
    ```
    step 1 : check the connection wires and make sure each one is tightened.
    step 2 : restart and setup the varjo.
    ```
    2. if meet the problem of camera in unity not moving with varjo headset, it is caused by **tracking in varjo is not working**, please restart the varjo in varjo base, and run setup.

    3. if it says streamVR is not installed in varjo headset, please **DO NOT INSTALL streamVR**, just restart again untill no warning happen.

    4. if it calls the issue of not find the XR dll or when run the calibration_example's calibration, it says no marker detected, please reinstall the package https://github.com/varjocom/VarjoUnityXRPlugin.git#3.4.0 again. And restart unity.



## Usage
There are three function you can use.
1) Clibration and object detection.
2) Text tooltip presentation.
3) progressBar.

Please follow the [README2](./readme2.md)

    
<!--
FIRST. make object of "VarjoApiManager.cs" and "TooltipManager.cs" and set in unity.

1. Show text always shown in front of camera:
```
1.Create empty object and name it as "text_example".
2. Drag Text_example.cs file to "text_example". And set the ref of Text_example in unity.
3. You can change the distance of text to camera, text size, color, sentences and the time to switch sentences.
```

2. Show progressbar 
```
1. Create empty object and name it as "progressBar".
2.Under  progressBar make a (UI-IMAGE)background, and (UI-IMAGE) backgroundfill(which fill the bar), and an empty object as points.
the structure:
progressBar
-  canvas
--    BG
--    BGFill
--    points
3. Set the backgroundfill sprite to "WhiteSprite_8x8", and set image type to filled, fill method to horizontal, and fill amount to 0, set native size. 
4. drag "pointpreFab" to progressBar's "point prefrab" setting.
4. Set the canvas position in front of main camera and rander mode to world space and event camera to main camera. 
5. Drag the "ProgressBarManager.cs" to progressBar, set all refs.
6. If you want to trigger the progressbar to run, use the function "PlayRange" in ProgressBarManager.cs. If you want to change the sentences list in other file, use function "SetSentences" or directly change in unity.
```
3. Calibration and Detection objects
```
1. Make empty object "calibration_example" and "receiver" and "sender". 
2. Drag "calibration_example.cs" to "calibration_example" and "DetectionReceiver.cs" to "reveiver" and "Sender.cs" to "sender" and setup.
2.Please put the headset in from of calibration markers, and open sender server in python part.
2. "calibration_example.cs" will do the calibration first, then connect the server.
3. You will see the 3d bboxes projection in varjo headset.
```
--!>
