@echo off
REM ===================================================================
REM  PLAY -- double-click to start AsteroidDefense (the Godot game).
REM
REM  1. builds the Rust physics library the game loads (fast when it is
REM     already up to date; the first build takes a few minutes)
REM  2. launches the game
REM
REM  Needs: Rust (cargo) and Godot 4.7 on PATH, plus the JPL kernels
REM  (de440s.bsp, pck11.pca). The game finds the kernels itself, or you
REM  can set ASTEROID_DE_KERNEL / ASTEROID_PLANETARY_CONSTANTS first.
REM ===================================================================
setlocal
title AsteroidDefense
cd /d "%~dp0"

REM Optional machine-local kernel paths (scripts\ is git-ignored).
if exist "scripts\_env.cmd" call "scripts\_env.cmd"

where cargo >nul 2>nul
if errorlevel 1 (
  echo [!] cargo not found on PATH. Install Rust from https://rustup.rs
  goto :fail
)

set "GODOT=godot"
where godot >nul 2>nul
if errorlevel 1 (
  if exist "%LOCALAPPDATA%\Microsoft\WinGet\Links\godot.exe" (
    set "GODOT=%LOCALAPPDATA%\Microsoft\WinGet\Links\godot.exe"
  ) else (
    echo [!] Godot not found on PATH. Install Godot 4.7 ^(winget install GodotEngine.GodotEngine^)
    goto :fail
  )
)

echo Building the physics library ^(debug -- the profile Godot loads^)...
cargo build -p asteroid_gdext
if errorlevel 1 (
  echo.
  echo [!] Build failed. If it says "Access is denied", a Godot window from an
  echo     earlier run is still open and holding the library -- close it and retry.
  goto :fail
)

echo.
echo Launching the game ^(first start imports assets and loads the ephemeris;
echo it can take a little while before the window is responsive^)...
"%GODOT%" --path godot
if errorlevel 1 goto :fail
endlocal
exit /b 0

:fail
echo.
pause
endlocal
exit /b 1
