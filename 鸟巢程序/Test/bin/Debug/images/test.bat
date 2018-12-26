@echo off
for %%i in (*.JPG) do (
  Xcopy  %%i  ..\00284.JPG /s /e /y
  cd ..
  test2018122401.exe 
  
  
  
  cd images
)
pause