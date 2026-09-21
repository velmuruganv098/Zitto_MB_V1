@echo off
"C:\Program Files\SEGGER\JLink\JLink.exe" -device S32K144 -if SWD -speed 1000 -CommanderScript "C:\Projects\Zitto_MB_V1\flash.jlink"