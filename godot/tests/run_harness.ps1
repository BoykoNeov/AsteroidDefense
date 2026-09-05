<#
.SYNOPSIS
  Run one of the godot/tests harnesses as a temporary autoload, then clean up.

.DESCRIPTION
  The harnesses in this folder (_shot.gd, _perf.gd, _pork_shot.gd, ...) only run
  when registered as an autoload named in project.godot, and the game must be
  launched non-headless for screenshots. Doing that by hand means editing
  project.godot, launching, and restoring the file - and, on this machine,
  remembering that Godot does not always exit on quit(): the editor plugin's
  background thread can leave the process alive at 100% CPU, which both poisons
  the next measurement and holds the debug DLL so `cargo build` fails with
  "Access is denied".

  This script does the whole ritual and kills only the PID it started.

.PARAMETER Harness
  Script under res://tests, e.g. _shot.gd or _perf.gd.

.PARAMETER Headless
  Run with --headless (no window; _draw still runs for visible nodes, frame
  times are uncapped - this is what _perf.gd wants for CPU cost).

.PARAMETER TimeoutSec
  Kill the run if it has not exited by then.

.EXAMPLE
  powershell -File godot/tests/run_harness.ps1 -Harness _shot.gd
  powershell -File godot/tests/run_harness.ps1 -Harness _perf.gd -Headless
#>
param(
    [Parameter(Mandatory = $true)] [string] $Harness,
    [switch] $Headless,
    [int] $TimeoutSec = 600,
    [string] $Log = ""
)

$ErrorActionPreference = "Stop"
$godotDir = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$project = Join-Path $godotDir "project.godot"
$name = ([IO.Path]::GetFileNameWithoutExtension($Harness)).TrimStart("_")
$name = $name.Substring(0, 1).ToUpper() + $name.Substring(1)   # _shot.gd -> Shot
$line = "$name=`"*res://tests/$Harness`""

# Explicit UTF-8 both ways. project.godot has non-ASCII in it (an em dash in the
# description); PowerShell 5.1's default read decodes a BOM-less file as the ANSI
# code page and the restore would write mangled bytes back.
$utf8 = New-Object System.Text.UTF8Encoding($false)
$original = [IO.File]::ReadAllText($project, $utf8)
if ($original -notmatch [regex]::Escape($line)) {
    # Register after Sim, so the harness's _ready runs after the autoload it drives.
    $patched = $original -replace '(?m)^(Sim="\*res://scripts/sim.gd")\r?$', "`$1`r`n$line"
    if ($patched -eq $original) { throw "could not find the Sim autoload line in $project" }
    # WriteAllText, not Set-Content -Encoding utf8: PowerShell 5.1 writes a BOM,
    # which Godot tolerates but git sees as a change to project.godot.
    [IO.File]::WriteAllText($project, $patched, $utf8)
}

$args = @("--path", $godotDir)
if ($Headless) { $args += "--headless" } else { $args += @("--resolution", "1600x900") }
if ($Log -eq "") {
    $Log = Join-Path $env:TEMP ("godot_harness_{0}_{1:yyyyMMdd_HHmmss}.log" -f $name, (Get-Date))
}

$launchedAt = Get-Date
try {
    $p = Start-Process -FilePath "godot" -ArgumentList $args -PassThru -NoNewWindow `
        -RedirectStandardOutput $Log -RedirectStandardError "$Log.err"
    Write-Host "harness $Harness started as PID $($p.Id), log $Log"
    if (-not $p.WaitForExit($TimeoutSec * 1000)) {
        Write-Warning "still running after $TimeoutSec s - killing PID $($p.Id)"
    }
    # quit() has returned but the process may linger (plugin teardown). Only the
    # PID this script started is ever touched.
    if (-not $p.HasExited) {
        Start-Sleep -Seconds 2
        if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force }
    }
}
finally {
    [IO.File]::WriteAllText($project, $original, $utf8)
    # `godot` on PATH is the WinGet shim: it launches the real engine as a child
    # and can exit while that child lingers at 100% CPU. Find the engine process
    # THIS run started - created after our launch, running THIS project directory
    # with THIS run's arguments - and stop it. Nothing else matches: another
    # project's Godot has another --path, and another run of this project would
    # have been started by another copy of this script.
    Start-Sleep -Seconds 1
    $mode = if ($Headless) { "--headless" } else { "--resolution" }
    Get-CimInstance Win32_Process |
        Where-Object {
            $_.Name -like "godot*" -and
            $_.CommandLine -like "*--path $godotDir*" -and
            $_.CommandLine -like "*$mode*" -and
            $_.CreationDate -ge $launchedAt
        } |
        ForEach-Object {
            Write-Warning "engine process $($_.ProcessId) from this run is still alive - stopping it"
            Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
        }
}

Get-Content $Log | Where-Object { $_ -match '^(SHOT|PERF|TIER2SHOT|PORKSHOT|THREATSHOT|TRACTORSHOT|PASS|FAIL|SCRIPT ERROR)' }
if (Test-Path "$Log.err") {
    Get-Content "$Log.err" | Where-Object { $_ -match 'SCRIPT ERROR|ERROR' } | Select-Object -First 20
}
