@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

:: ==========================================================================
:: VoxelWorld ray-compare collection script
:: Single VTune hotspots collection for:
::   voxel_world.exe --ray-compare --invokers 100 --frames 100 --rays-per-invoker 20 --seed 42
::
:: The same ray is issued back-to-back on mesh then grid inside one process;
:: VTune attributes the cost to the call trees (rayTest vs fieldRay).
:: Run as administrator for hardware event sampling.
:: ==========================================================================

:: Script lives in scripts\, project root is one level up
set "PROJECT=%~dp0.."
for %%I in ("%PROJECT%") do set "PROJECT=%%~fI"

set "SET_VTUNE=C:\Program Files (x86)\Intel\oneAPI\setvars.bat"
set "EXE_PATH=%PROJECT%\out\build\x64-Release\voxel_world.exe"
set "WORK_DIR=%PROJECT%\out\build\x64-Release"
set "RESULT_DIR=%PROJECT%\vtune-results\ray-compare"

call "%SET_VTUNE%" >nul 2>&1

if not exist "%EXE_PATH%" (
    echo ERROR: exe not found: %EXE_PATH%
    echo        Build x64-Release RelWithDebInfo first.
    pause
    exit /b 1
)

echo Try kill residual vtune/pin process...
taskkill /f /im vtune.exe >nul 2>&1
taskkill /f /im pin.exe >nul 2>&1
timeout /t 1 /nobreak >nul

if exist "%RESULT_DIR%" rmdir /s /q "%RESULT_DIR%"

cd /d "%WORK_DIR%"
echo ======================================
echo Collect ray-compare [hotspots] ...
echo ======================================
vtune -collect hotspots ^
    -result-dir="%RESULT_DIR%" ^
    -search-dir="%WORK_DIR%" ^
    -search-dir="%PROJECT%\out\build\x64-Release" ^
    -- "%EXE_PATH%" --ray-compare --invokers 75 --frames 1000 --rays-per-invoker 20 --compare grid
if errorlevel 1 (
    echo Collection failed.
    pause
    exit /b 1
)

echo.
echo ======================================
echo Collection finished.
echo Open: vtune-gui "%RESULT_DIR%"
echo ======================================
pause
