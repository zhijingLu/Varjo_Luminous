@echo off
setlocal EnableDelayedExpansion

REM ====== setting======
set "V02_DIR=C:\path\to\V02"
set "OPENCV_EDIT_DIR=C:\path\to\opencv_edit"

set "VARJO_INC=C:\path\to\Varjo_SDK_for_Custom_Engines_4.12.0\varjo-sdk\include"
set "VARJO_LIB=C:\path\to\Varjo_SDK_for_Custom_Engines_4.12.0\varjo-sdk\lib"

set "OPENCV_INC=C:\path\to\opencv\build\include"
set "OPENCV_LIB=C:\path\to\opencv\build\x64\vc17\lib"
set "OPENCV_LINK_LIBS=opencv_core480.lib opencv_imgproc480.lib opencv_imgcodecs480.lib opencv_highgui480.lib opencv_calib3d480.lib opencv_features2d480.lib opencv_flann480.lib opencv_aruco480.lib opencv_ccalib480.lib"





REM ====== get all v02  cpp + opencv_edit\opencv.cpp ======
set SRCS=
for /r "%V02_DIR%" %%f in (*.cpp) do (
  set SRCS=!SRCS! "%%f"
)
set SRCS=!SRCS! "%OPENCV_EDIT_DIR%\opencv.cpp"

set OUT_DIR=%cd%\build
if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"
pushd "%OUT_DIR%"


echo [INFO] Sources: !SRCS!

REM ====== compile ======
cl /nologo /EHsc /MD /std:c++17 /LD ^
  /D VARJOSTREAM_EXPORTS ^
  /I"%V02_DIR%" ^
  /I"%OPENCV_EDIT_DIR%" ^
  /I"%VARJO_INC%" ^
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

















