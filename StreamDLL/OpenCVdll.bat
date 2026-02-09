@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM ==========SETTING=============

set "OPENCV_EDIT_DIR=D:\path\to\opencv_edit"   
set "OUT_DIR=D:\path\to\build\opencv_edit"    
set "OPENCV_INC=D:\path\to\opencv\build\include"
set "OPENCV_LIB=D:\path\to\opencv\build\x64\vc17\lib"
set "OPENCV_BIN=D:\path\to\opencv\build\x64\vc17\bin"

set "OPENCV_LINK_LIBS=opencv_world490.lib"



if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"


REM ======Compile=================

set "CFLAGS=/nologo /std:c++17 /EHsc /MD /O2"
set "LDFLAGS=/nologo /DLL"

if "%DEBUG%"=="1" (
  set "CFLAGS=/nologo /std:c++17 /EHsc /MDd /Zi /Od"
  set "LDFLAGS=/nologo /DLL /DEBUG"
)

cl %CFLAGS% ^
  /I "%OPENCV_EDIT_DIR%" ^
  /I "%OPENCV_INC%" ^
  "%OPENCV_EDIT_DIR%\opencv.cpp" ^
  /Fo"%OUT_DIR%\opencv_edit.obj" ^
  /Fe"%OUT_DIR%\opencv_edit.dll" ^
  /Fd"%OUT_DIR%\opencv_edit.pdb" ^
  /link %LDFLAGS% ^
  /LIBPATH:"%OPENCV_LIB%" %OPENCV_LINK_LIBS% ^
  /OUT:"%OUT_DIR%\OpenCV.dll" ^
  /IMPLIB:"%OUT_DIR%\OpenCV.lib"

if errorlevel 1 (
  echo [ERR] build dll failed
  exit /b 1
)


echo [OK] Built:
echo   %OUT_DIR%\opencv_edit.dll
echo   %OUT_DIR%\opencv_edit.lib
endlocal
