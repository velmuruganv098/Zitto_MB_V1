@echo off

set F1=C:\Users\Velu\workspaceS32DS.3.4\Zitto_MB_V1\Debug_FLASH\Project_Settings\Startup_Code\startup_S32K144.args
set F2=C:\Users\Velu\workspaceS32DS.3.4\Zitto_MB_V1\Debug_FLASH\RTT\SEGGER_RTT_ASM_ARMv7M.args

if exist "%F1%" (
    findstr /x /c:"-c" "%F1%" >nul 2>&1
    if errorlevel 1 (
        (echo -c) > "%F1%.tmp"
        type "%F1%" >> "%F1%.tmp"
        move /y "%F1%.tmp" "%F1%" >nul
    )
)

if exist "%F2%" (
    findstr /x /c:"-c" "%F2%" >nul 2>&1
    if errorlevel 1 (
        (echo -c) > "%F2%.tmp"
        type "%F2%" >> "%F2%.tmp"
        move /y "%F2%.tmp" "%F2%" >nul
    )
)