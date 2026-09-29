<#
.SYNOPSIS
    Sets up Drone Show Studio from source: Python environment, both C++ engines, the desktop app.

.DESCRIPTION
    1. Checks the prerequisites and lists anything missing (it never installs them).
    2. Creates .venv with Python 3.14 and installs requirements-dev.txt.
    3. Opens a Visual Studio developer shell, then configures, builds and tests
       stage2_core_engine and stage3_simulation_packer with their "release" CMake presets.
       vcpkg installs the C++ libraries from vcpkg.json into vcpkg_installed\ on the first run.
    4. Runs the Studio dependency check and the Stage 2 smoke test.
    5. Installs the desktop app's npm packages (npm ci).

    Safe to run again: finished steps are quick the second time.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File .\setup.ps1
.EXAMPLE
    .\setup.ps1 -Python C:\Python314\python.exe -FullTests
#>
[CmdletBinding()]
param(
    # Python 3.14 used to create .venv. Default: the "py -3.14" launcher, then "python" on PATH.
    [string]$Python,
    # Skip the desktop app (Node.js and Rust are then not required).
    [switch]$SkipApp,
    # Skip the C++ unit tests and the Stage 2 smoke test.
    [switch]$SkipTests,
    # Also run the whole pytest suite (about 3 minutes).
    [switch]$FullTests,
    # Reconfigure both engines from scratch (cmake --fresh).
    [switch]$Clean
)

$ErrorActionPreference = 'Stop'
$Repo = $PSScriptRoot
$RequiredPython = '3.14'
$MinNodeMajor = 20

function Write-Step([string]$Text) { Write-Host ''; Write-Host "==> $Text" -ForegroundColor Cyan }
function Write-Ok([string]$Text) { Write-Host "    ok    $Text" -ForegroundColor Green }
function Write-Bad([string]$Text) { Write-Host "    MISSING $Text" -ForegroundColor Red }

# Windows PowerShell 5.1 turns a native program's redirected stderr into a terminating error
# when $ErrorActionPreference is Stop, so native programs run with Continue and are judged
# by their exit code instead.

# Runs a native command and stops the script when it fails.
function Invoke-Checked([string]$What, [scriptblock]$Command) {
    $old = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try { & $Command } finally { $ErrorActionPreference = $old }
    if ($LASTEXITCODE -ne 0) { throw "$What failed (exit code $LASTEXITCODE)." }
}

# Runs a native command with its stderr discarded and returns its output.
function Invoke-Quiet([scriptblock]$Command) {
    $old = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try { & $Command 2>$null } finally { $ErrorActionPreference = $old }
}

function Get-PythonVersion([string]$Exe) {
    Invoke-Quiet { & $Exe -c "import sys; print('%d.%d' % sys.version_info[:2])" }
}

Set-Location $Repo

# ---------------------------------------------------------------- 1. prerequisites
Write-Step 'Checking prerequisites'
$missing = @()

if (Get-Command git -ErrorAction SilentlyContinue) { Write-Ok 'Git' }
else { $missing += 'Git: https://git-scm.com/download/win' }

$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$vsPath = $null
if (Test-Path $vswhere) {
    $vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
}
if ($vsPath) {
    Write-Ok "Visual Studio C++ tools ($vsPath)"
    if (-not (Test-Path (Join-Path $vsPath 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'))) {
        $missing += 'Visual Studio component "C++ CMake tools for Windows" (Visual Studio Installer > Modify > Individual components)'
    }
    if (-not $env:VCPKG_ROOT -and -not (Test-Path (Join-Path $vsPath 'VC\vcpkg\vcpkg.exe'))) {
        $missing += 'Visual Studio component "vcpkg package manager" (or set VCPKG_ROOT to your own vcpkg clone)'
    }
} else {
    $missing += 'Visual Studio 2022 or later (Community or Build Tools) with the "Desktop development with C++" workload: https://visualstudio.microsoft.com/downloads/'
}

$pythonExe = $null
if ($Python) {
    $pythonExe = $Python
} elseif (Get-Command py -ErrorAction SilentlyContinue) {
    $pythonExe = Invoke-Quiet { & py "-$RequiredPython" -c "import sys; print(sys.executable)" }
} elseif (Get-Command python -ErrorAction SilentlyContinue) {
    $pythonExe = (Get-Command python).Source
}
$venvPython = Join-Path $Repo '.venv\Scripts\python.exe'
if (Test-Path $venvPython) {
    $v = Get-PythonVersion $venvPython
    if ($v -eq $RequiredPython) { Write-Ok "Python $v (existing .venv)" }
    else { $missing += "Python $RequiredPython in .venv (it has Python $v). Delete .venv and run setup again." }
} elseif ($pythonExe -and ((Get-PythonVersion $pythonExe) -eq $RequiredPython)) {
    Write-Ok "Python $RequiredPython ($pythonExe)"
} else {
    $missing += "Python $RequiredPython (64-bit): https://www.python.org/downloads/ (or pass -Python <path to python.exe>)"
}

if (-not $SkipApp) {
    $node = Get-Command node -ErrorAction SilentlyContinue
    if ($node) {
        $nodeVersion = (& node -v).TrimStart('v')
        if ([int]($nodeVersion.Split('.')[0]) -ge $MinNodeMajor) { Write-Ok "Node.js $nodeVersion" }
        else { $missing += "Node.js $MinNodeMajor or later (found $nodeVersion): https://nodejs.org/" }
    } else {
        $missing += 'Node.js (LTS): https://nodejs.org/'
    }
    if (Get-Command cargo -ErrorAction SilentlyContinue) { Write-Ok ((& cargo -V) -join '') }
    else { $missing += 'Rust (for the Tauri app): https://rustup.rs/' }
}

if ($missing.Count -gt 0) {
    Write-Host ''
    Write-Host 'Install these, then run setup.ps1 again:' -ForegroundColor Yellow
    foreach ($m in $missing) { Write-Bad $m }
    exit 1
}

# ---------------------------------------------------------------- 2. Python environment
Write-Step 'Python environment (.venv)'
if (-not (Test-Path $venvPython)) {
    Invoke-Checked 'Creating .venv' { & $pythonExe -m venv (Join-Path $Repo '.venv') }
    Write-Ok 'created .venv'
}
Invoke-Checked 'pip install' { & $venvPython -m pip install --disable-pip-version-check -q -r (Join-Path $Repo 'requirements-dev.txt') }
Write-Ok 'packages from requirements-dev.txt installed'

# ---------------------------------------------------------------- 3. C++ engines
Write-Step 'Visual Studio developer shell'
$userVcpkgRoot = $env:VCPKG_ROOT
# The dev shell script writes harmless noise to stderr.
Invoke-Quiet { & (Join-Path $vsPath 'Common7\Tools\Launch-VsDevShell.ps1') -Arch amd64 -HostArch amd64 -SkipAutomaticLocation } | Out-Null
if ($userVcpkgRoot) { $env:VCPKG_ROOT = $userVcpkgRoot }   # keep a vcpkg the user chose
Set-Location $Repo
if (-not (Get-Command cl -ErrorAction SilentlyContinue)) { throw 'The developer shell did not provide the C++ compiler (cl.exe).' }
Write-Ok "compiler, CMake and Ninja ready; vcpkg: $env:VCPKG_ROOT"

$manifestInstall = (Join-Path $Repo 'vcpkg_installed').Replace('\', '/')
foreach ($engine in @('stage2_core_engine', 'stage3_simulation_packer')) {
    Write-Step "Building $engine"
    Push-Location (Join-Path $Repo $engine)
    try {
        # An old build folder configured without the preset (or with another vcpkg) is reconfigured from scratch.
        $cache = Join-Path 'build' 'CMakeCache.txt'
        $fresh = $Clean -or -not (Test-Path $cache) -or -not (Select-String -Path $cache -SimpleMatch "VCPKG_INSTALLED_DIR:PATH=$manifestInstall" -Quiet)
        if ($fresh) {
            Write-Host '    configuring (the first run also builds the C++ libraries; a few minutes)'
            Invoke-Checked "Configuring $engine" { cmake --preset release --fresh --log-level=WARNING }
        } else {
            Invoke-Checked "Configuring $engine" { cmake --preset release --log-level=WARNING }
        }
        Invoke-Checked "Building $engine" { cmake --build --preset release }
        Write-Ok 'built'
        if (-not $SkipTests) {
            Invoke-Checked "Testing $engine" { ctest --preset release }
            Write-Ok 'unit tests passed'
        }
    } finally {
        Pop-Location
    }
}

# ---------------------------------------------------------------- 4. checks
Write-Step 'Checking the pipeline'
# Native libraries (OpenCL drivers) may print to stderr; only the event on stdout matters.
$doctorLine = Invoke-Quiet { & $venvPython -m tools.studio_bridge doctor } | Where-Object { $_ -like '{"type": "doctor"*' -or $_ -like '{"type":"doctor"*' } | Select-Object -First 1
if (-not $doctorLine) { throw 'The Studio dependency check (python -m tools.studio_bridge doctor) gave no result.' }
$doctor = $doctorLine | ConvertFrom-Json
$failed = $false
foreach ($check in $doctor.checks) {
    if ($check.ok) { Write-Ok ("{0,-15} {1}" -f $check.name, $check.detail) }
    else { Write-Bad ("{0,-15} {1} (needed for {2})" -f $check.name, $check.detail, $check.required_for); $failed = $true }
}
if ($failed) { throw 'The dependency check found a problem (see above).' }

if (-not $SkipTests) {
    Invoke-Checked 'Stage 2 smoke test' { & $venvPython (Join-Path $Repo 'tools\scripts\smoke_test_stage2.py') }
    Write-Ok 'Stage 2 smoke test passed'
}
if ($FullTests) {
    Invoke-Checked 'pytest' { & $venvPython -m pytest -q (Join-Path $Repo 'tests') }
    Write-Ok 'pytest suite passed'
}

# ---------------------------------------------------------------- 5. desktop app
if (-not $SkipApp) {
    Write-Step 'Desktop app packages (apps\)'
    Push-Location (Join-Path $Repo 'apps')
    try {
        Invoke-Checked 'npm ci' { npm ci --no-audit --no-fund }
        Write-Ok 'npm packages installed'
    } finally {
        Pop-Location
    }
}

Write-Host ''
Write-Host 'Setup complete.' -ForegroundColor Green
if (-not $SkipApp) {
    Write-Host 'Start the app:   cd apps; npm run tauri dev'
    Write-Host 'Build installer: cd apps; npm run tauri build'
}
