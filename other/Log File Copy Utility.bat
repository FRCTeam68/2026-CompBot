@echo off
setlocal

cls
echo =====================================
echo        Log File Copy Utility
echo =====================================
echo.

:: Ask for destination folder once
set /p DESTFOLDER=Enter destination folder name: 

if "%DESTFOLDER%"=="" set DESTFOLDER=unknown

:: Remove quotes if user typed them
set DESTFOLDER=%DESTFOLDER:"=%

:: Remove trailing backslash if present
if "%DESTFOLDER:~-1%"=="\" set DESTFOLDER=%DESTFOLDER:~0,-1%

:: Build destination path
set DEST=%~dp0%DESTFOLDER%

:: Create folder if missing
if not exist "%DEST%" mkdir "%DEST%"

:START
cls
echo =====================================
echo        Log File Copy Utility
echo =====================================
echo.
echo Destination folder: %DESTFOLDER%
echo.

echo Please plug in the flash drive.
pause

echo.
echo Available Drives:
echo ----------------------------------------
wmic logicaldisk get name, volumename
echo ----------------------------------------
echo.

:: Ask for drive letter every cycle
set "DRIVE="
set /p DRIVE=Enter the flash drive letter (example: h): 
set DRIVE=%DRIVE::=%

set SOURCE=%DRIVE%:\logs

if not exist "%SOURCE%" goto BADDRIVE

echo.
echo Copying contents of:
echo   %SOURCE%
echo To:
echo   %DEST%
echo.

:: Copy only contents, preserve timestamps
robocopy "%SOURCE%" "%DEST%\." /E /COPY:DAT /R:2 /W:2

echo.
echo Copy operation complete.
echo.

set "WIPE="
set /p WIPE=Delete logs folder from flash drive? Type yes to confirm: 

if /I "%WIPE%"=="yes" goto WIPELOGS
goto SKIPWIPE

:BADDRIVE
echo.
echo ERROR: %SOURCE% not found.
pause
goto START

:WIPELOGS
echo.
echo Deleting %SOURCE% ...
rmdir /S /Q "%SOURCE%"
echo Logs folder deleted.
goto AFTERWIPE

:SKIPWIPE
echo.
echo Flash drive was NOT modified.

:AFTERWIPE
echo.
echo Restarting...
timeout /t 1 >nul
goto START