@echo off
REM VCU Master - run.py creates .venv, installs / updates the dependencies and starts the app.
cd /d "%~dp0"
where py >nul 2>&1 && (py -3 run.py %*) || (python run.py %*)
pause
