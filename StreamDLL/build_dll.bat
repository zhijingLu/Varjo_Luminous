@echo off
setlocal EnableDelayedExpansion

REM ====== setting======
set "ROOT=C:\Users\zlu\Downloads\Varjo_Luminous-main\Varjo_Luminous-main\StreamDLL"
set "V02_DIR=%ROOT%\V02"
set "OPENCV_EDIT_DIR=%ROOT%\Opencv"
set "VARJO_INC=C:\Users\zlu\Desktop\project\Varjo_Experimental_SDK_for_Custom_Engines_4.12.0\varjo-sdk-experimental\include"
set "VARJO_INC_EX=C:\Users\zlu\Desktop\project\Varjo_Experimental_SDK_for_Custom_Engines_4.12.0\varjo-sdk-experimental\include_experimental"

set "VARJO_LIB=C:\Users\zlu\Desktop\project\Varjo_Experimental_SDK_for_Custom_Engines_4.12.0\varjo-sdk-experimental\lib"
set "OPENCV_INC=C:\Users\zlu\Desktop\Talha\install\include"
set "OPENCV_LIB=C:\Users\zlu\Desktop\Talha\install\lib"

set "OPENCV_LINK_LIBS="
for %%L in ("%OPENCV_LIB%\opencv_*.lib") do (
  set "OPENCV_LINK_LIBS=!OPENCV_LINK_LIBS! %%~nxL"
)





REM ====== get all v02  cpp+Opencv ======
set "SRCS="
for /r "%V02_DIR%" %%f in (*.cpp) do (
  set SRCS=!SRCS! "%%f"
)
for /r "%OPENCV_EDIT_DIR%" %%f in (*.cpp) do (
  set SRCS=!SRCS! "%%f"
)

set "OUT_DIR=%ROOT%\build\VarjoStream"
if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"
pushd "%OUT_DIR%"

echo [INFO] Sources: !SRCS!
echo [INFO] OPENCV_LINK_LIBS: !OPENCV_LINK_LIBS!


REM ====== compile ======
cl /nologo /EHsc /MD /O2 /std:c++17 /LD /GL ^
  /D VARJOSTREAM_EXPORTS ^
  /I"%V02_DIR%" ^
  /I"%OPENCV_EDIT_DIR%" ^
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

















