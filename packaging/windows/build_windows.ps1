# PowerShell build script for Windows x86_64:
#   1. PyInstaller onedir build (packaging/work/dist/pyCub)
#   2. Portable ZIP archive (packaging/out/pyCub-<version>-Portable-Windows-x64.zip)
#   3. Inno Setup installer (packaging/out/pyCub-Setup-<version>-Windows-x64.exe)
$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Resolve-Path (Join-Path $ScriptDir "..\..")
$VersionFile = Join-Path $RepoRoot "studio\VERSION"
$DefaultVersion = if (Test-Path $VersionFile) { (Get-Content $VersionFile -Raw).Trim() } else { "1.4.0" }
$Version = if ($env:PYCUB_VERSION) { $env:PYCUB_VERSION } else { $DefaultVersion }
$WorkDir = Join-Path $RepoRoot "packaging\work"
$DistDir = Join-Path $WorkDir "dist"
$BuildDir = Join-Path $WorkDir "build"
$OutDir = Join-Path $RepoRoot "packaging\out"

New-Item -ItemType Directory -Force -Path $DistDir, $BuildDir, $OutDir | Out-Null

Write-Host "==> [1/3] Running PyInstaller..."
python -m PyInstaller --noconfirm --clean `
  --distpath "$DistDir" `
  --workpath "$BuildDir" `
  (Join-Path $RepoRoot "packaging\pycub.spec")

Write-Host "==> [2/3] Creating Portable ZIP..."
$ZipPath = Join-Path $OutDir "pyCub-$Version-Portable-Windows-x64.zip"
if (Test-Path $ZipPath) { Remove-Item $ZipPath -Force }
Compress-Archive -Path (Join-Path $DistDir "pyCub") -DestinationPath $ZipPath

Write-Host "==> [3/3] Building Inno Setup .exe installer..."
$IsccCandidates = @(
  "iscc",
  "C:\Program Files (x86)\Inno Setup 6\ISCC.exe",
  "C:\Program Files\Inno Setup 6\ISCC.exe"
)
$IsccExe = $null
foreach ($cand in $IsccCandidates) {
  if (Get-Command $cand -ErrorAction SilentlyContinue) {
    $IsccExe = $cand
    break
  }
  if (Test-Path $cand) {
    $IsccExe = $cand
    break
  }
}
if ($IsccExe) {
  & $IsccExe "/DMyAppVersion=$Version" (Join-Path $ScriptDir "installer.iss")
} else {
  Write-Warning "Inno Setup (ISCC.exe) not found; skipped Setup .exe generation."
}

Get-ChildItem $OutDir
