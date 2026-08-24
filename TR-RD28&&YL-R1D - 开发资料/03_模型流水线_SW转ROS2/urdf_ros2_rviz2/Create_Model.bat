@echo off
chcp 65001 >nul
REM ===================================================
REM Create_Model.bat - Windows Batch Wrapper
REM
REM Usage:
REM   Create_Model.bat <source_dir> <new_name> [options]
REM
REM Examples:
REM   Create_Model.bat Robot ylr1d
REM   Create_Model.bat Robot ylr1d -o ./src -v
REM   Create_Model.bat Robot ylr1d --no-xacro
REM ===================================================

set SOURCE_DIR=%1
set NEW_NAME=%2

if "%SOURCE_DIR%"=="" (
    echo Usage: Create_Model.bat ^<source_dir^> ^<new_name^> [options]
    echo.
    echo Examples:
    echo   Create_Model.bat Robot ylr1d
    echo   Create_Model.bat Robot ylr1d -v --no-xacro
    echo.
    pause
    exit /b 1
)

if "%NEW_NAME%"=="" (
    echo Error: new_name is required
    echo Usage: Create_Model.bat ^<source_dir^> ^<new_name^> [options]
    pause
    exit /b 1
)

REM Check Python availability
python --version >nul 2>&1
if errorlevel 1 (
    echo Error: Python not found. Please install Python 3.8+ and ensure it is in your PATH.
    pause
    exit /b 1
)

REM Build remaining args (skip %1 and %2). shift does NOT affect %* in batch.
set REMAINING=
set _idx=0
setlocal enabledelayedexpansion
for %%a in (%*) do (
    set /a _idx+=1
    if !_idx! gtr 2 (
        if defined REMAINING (
            set "REMAINING=!REMAINING! %%a"
        ) else (
            set "REMAINING=%%a"
        )
    )
)

REM Run the pipeline
echo ===================================================
echo Model Pipeline
echo   Source: %SOURCE_DIR%
echo   New Name: %NEW_NAME%
echo   Extra Args: !REMAINING!
echo ===================================================
echo.

python -m model_pipeline -s "%SOURCE_DIR%" -n "%NEW_NAME%" !REMAINING!
endlocal

if errorlevel 1 (
    echo.
    echo Pipeline failed with error code %errorlevel%
    pause
    exit /b %errorlevel%
)

echo.
echo Pipeline completed successfully!
pause
