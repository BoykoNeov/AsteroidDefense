@echo off
REM ===================================================================
REM  START -- double-click to build and launch the Asteroid Deflection
REM  Simulator (the Godot 3D frontend).
REM
REM  What it does:
REM    1. checks that Rust (cargo) and Godot 4.7 are installed
REM    2. checks that the JPL kernels are in the repo's kernels\ folder
REM       and offers to download them if they are not
REM    3. builds the Rust physics binding (the DLL Godot loads)
REM    4. starts the game
REM
REM  Godot is found on PATH as "godot", or set GODOT to the full path of
REM  your Godot 4.7 executable before running this file.
REM ===================================================================
setlocal
title Asteroid Deflection Simulator
cd /d "%~dp0"

REM ---- 1. tools -----------------------------------------------------
where cargo >nul 2>&1
if errorlevel 1 (
  echo [!] Rust is not installed ^(cargo not found^).
  echo     Install it from https://rustup.rs and run this file again.
  goto :fail
)

if not defined GODOT set "GODOT=godot"
where "%GODOT%" >nul 2>&1
if errorlevel 1 if not exist "%GODOT%" (
  echo [!] Godot 4.7 not found.
  echo     Install it ^(e.g. "winget install GodotEngine.GodotEngine"^),
  echo     or set GODOT to the full path of your Godot executable.
  goto :fail
)

REM ---- 2. kernels ---------------------------------------------------
REM The game needs a DE440 ephemeris and the planetary-constants file.
REM Either de440s.bsp or the full de440.bsp is accepted.
set "HAVE_DE=0"
if exist "kernels\de440s.bsp" set "HAVE_DE=1"
if exist "kernels\de440.bsp" set "HAVE_DE=1"
set "HAVE_PCA=0"
if exist "kernels\pck11.pca" set "HAVE_PCA=1"

if "%HAVE_DE%%HAVE_PCA%"=="11" goto :build

echo The JPL kernels are missing from:
echo     %~dp0kernels
echo.
choice /c YN /m "Download them now (about 35 MB, needs Python)"
if errorlevel 2 goto :fail
where python >nul 2>&1
if errorlevel 1 (
  echo [!] Python not found. Install Python 3 or run
  echo       python tools\fetch_kernels.py
  echo     yourself, then start again.
  goto :fail
)
python tools\fetch_kernels.py --neo
if errorlevel 1 (
  echo [!] The kernel download failed -- see the messages above.
  goto :fail
)

REM ---- 3. build -----------------------------------------------------
:build
echo Building the physics binding...
echo   ^(the first build takes a few minutes; later starts take seconds^)
echo.
REM Debug profile on purpose: that is the DLL Godot loads when it runs the
REM project (see godot\asteroid.gdextension). The physics core itself is
REM optimised even in this profile.
cargo build -p asteroid_gdext
if errorlevel 1 (
  echo.
  echo [!] The build failed. If the error says "Access is denied", the game
  echo     is probably still open from an earlier run -- close it and try again.
  goto :fail
)

REM ---- 4. launch ----------------------------------------------------
echo.
echo Starting the simulator... ^(loading the solar system takes a few
echo seconds after the window opens; the keys are listed on screen^)
start "" "%GODOT%" --path "%~dp0godot"
endlocal
exit /b 0

:fail
echo.
pause
endlocal
exit /b 1
