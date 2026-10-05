@echo off
rem Launches the supervision of the robots on Windows: bin\supervision --help
rem
rem The first time, creates the Python environment .venv at the root of the repository and installs
rem the dependencies of supervision\requirements.txt in it: nothing is installed anywhere else.
setlocal
set "REPO=%~dp0.."
set "VENV=%REPO%\.venv"
set "REQUIREMENTS=%REPO%\supervision\requirements.txt"

if not exist "%VENV%\Scripts\python.exe" (
    echo Creating the Python environment %VENV% 1>&2
    py -3 -m venv "%VENV%" || python -m venv "%VENV%" || exit /b 1
)
rem First time, or dependencies changed
fc /b "%REQUIREMENTS%" "%VENV%\requirements.txt" >nul 2>&1
if errorlevel 1 (
    echo Installing the dependencies of the supervision 1>&2
    "%VENV%\Scripts\python.exe" -m pip install --quiet --disable-pip-version-check -r "%REQUIREMENTS%" || exit /b 1
    copy /y "%REQUIREMENTS%" "%VENV%\requirements.txt" >nul
)
set "PYTHONPATH=%REPO%"
"%VENV%\Scripts\python.exe" -m supervision %*
