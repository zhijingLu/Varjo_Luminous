@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM ==========SETTING=============

set "OPENCV_EDIT_DIR=C:path\to\StreamDLL\Opencv"   
set "OUT_DIR=C:path\to\StreamDLL\build\Opencv"    
set "OPENCV_INC=C:path\to\install\include"
set "OPENCV_LIB=C:path\to\install\lib"

REM Build OPENCV_LINK_LIBS from all opencv_*.lib in OPENCV_LIB
set "OPENCV_LINK_LIBS="
for %%L in ("%OPENCV_LIB%\opencv_*.lib") do (
  set "OPENCV_LINK_LIBS=!OPENCV_LINK_LIBS! %%~nxL"
)



if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"

REM ======Compile=================

set "CFLAGS=/nologo /std:c++17 /EHsc /MD /O2 /GL"
set "LDFLAGS=/nologo /DLL"

if "%DEBUG%"=="1" (
  set "CFLAGS=/nologo /std:c++17 /EHsc /MDd /Zi /Od"
  set "LDFLAGS=/nologo /DLL /DEBUG"
)

cl %CFLAGS% ^
  /D VARJOSTREAM_EXPORTS ^
  /I "%OPENCV_EDIT_DIR%" ^
  /I "%OPENCV_INC%" ^
  "%OPENCV_EDIT_DIR%\opencv.cpp" ^
  /Fo"%OUT_DIR%\opencv.obj" ^
  /Fe"%OUT_DIR%\opencv.dll" ^
  /Fd"%OUT_DIR%\opencv.pdb" ^
  /link %LDFLAGS% ^
  /LIBPATH:"%OPENCV_LIB%" !OPENCV_LINK_LIBS! ^
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
