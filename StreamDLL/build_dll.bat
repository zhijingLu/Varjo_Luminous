@echo off
setlocal EnableDelayedExpansion

REM ====== setting======
set "V02_DIR=C:\Users\zlu\Downloads\Varjo_Luminous-main\Varjo_Luminous-main\StreamDLL\V02"

set "VARJO_INC=C:\Users\zlu\Desktop\project\Varjo_Experimental_SDK_for_Custom_Engines_4.12.0\varjo-sdk-experimental\include"
set "VARJO_INC_EX=C:\Users\zlu\Desktop\project\Varjo_Experimental_SDK_for_Custom_Engines_4.12.0\varjo-sdk-experimental\include_experimental"

set "VARJO_LIB=C:\Users\zlu\Desktop\project\Varjo_Experimental_SDK_for_Custom_Engines_4.12.0\varjo-sdk-experimental\lib"
set "OPENCV_INC=C:\Users\zlu\Desktop\Talha\install\include"
set "OPENCV_LIB=C:\Users\zlu\Desktop\Talha\install\lib"
set "OPENCV_LINK_LIBS=opencv_core480.lib opencv_imgproc480.lib opencv_imgcodecs480.lib opencv_highgui480.lib opencv_calib3d480.lib opencv_features2d480.lib opencv_flann480.lib opencv_aruco480.lib opencv_ccalib480.lib"





REM ====== get all v02  cpp ======
set SRCS=
for /r "%V02_DIR%" %%f in (*.cpp) do (
  set SRCS=!SRCS! "%%f"
)

set OUT_DIR=%cd%\build
if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"
pushd "%OUT_DIR%"


echo [INFO] Sources: !SRCS!


REM ====== compile ======
cl /nologo /EHsc /MD /O2 /std:c++17 /LD ^
  /D VARJOSTREAM_EXPORTS ^
  /I"%V02_DIR%" ^
  /I"%VARJO_INC%" ^
  /I"%VARJO_INC_EX%" ^
  /I"%OPENCV_INC%" ^
  !SRCS! ^
  /link ^
  /LIBPATH:"%VARJO_LIB%" ^
  /LIBPATH:"%OPENCV_LIB%" ^
  VarjoLib.lib %OPENCV_LINK_LIBS% ^
  /OUT:VarjoStream.dll ^
  /IMPLIB:VarjoStream.lib

popd
endlocal

















