@echo off
rem Flash the node firmware to a XIAO ESP32-C6:  flash-node COM9
if "%1"=="" (echo usage: flash-node COMx & exit /b 1)
cd /d "%~dp0..\node"
call "%~dp0..\..\idf.cmd" -p %1 flash
