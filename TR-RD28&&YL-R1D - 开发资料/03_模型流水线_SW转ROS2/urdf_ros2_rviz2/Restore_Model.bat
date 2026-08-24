@echo off
chcp 65001 >nul
REM ===================================================
REM Restore_Model.bat - Restore a model from its backup
REM
REM Usage:
REM   Restore_Model.bat <model_name>
REM
REM Examples:
REM   Restore_Model.bat Robot       (restores from Robot_copy)
REM   Restore_Model.bat YLR1D       (restores from YLR1D_copy)
REM ===================================================

set MODEL_NAME=%1

if "%MODEL_NAME%"=="" (
    echo Usage: Restore_Model.bat ^<model_name^>
    echo.
    echo Restores a model directory from its _copy backup.
    echo.
    echo Examples:
    echo   Restore_Model.bat Robot
    echo   Restore_Model.bat YLR1D
    echo.
    pause
    exit /b 1
)

set BACKUP_DIR="%MODEL_NAME%_copy"

if not exist %BACKUP_DIR% (
    echo Error: Backup directory %BACKUP_DIR% not found.
    echo Nothing to restore.
    pause
    exit /b 1
)

if exist "%MODEL_NAME%" (
    echo Removing current model directory: %MODEL_NAME%
    rmdir /s /q "%MODEL_NAME%"
)

echo Restoring from backup: %BACKUP_DIR% -^> %MODEL_NAME%
robocopy %BACKUP_DIR% "%MODEL_NAME%" /E /NJH /NJS /NP 2>nul
if errorlevel 8 (
    echo Error: Failed to restore from backup.
    pause
    exit /b 1
)

echo Restore complete! Model directory: %MODEL_NAME%
pause
