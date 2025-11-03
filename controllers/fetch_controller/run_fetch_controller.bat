@echo off
REM Activate the conda environment
CALL "C:\Users\Liraz Calif\anaconda3\Scripts\activate.bat" label-studio-gpu

REM Run your controller
python "C:\Users\Liraz Calif\Documents\devices\controllers\fetch_controller.py"

REM Pause to see errors
pause
