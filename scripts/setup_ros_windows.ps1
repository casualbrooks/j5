param(
  [switch]$Rebuild
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path | Split-Path -Parent
$ws = Join-Path $root 'ros_ws'
Write-Host "Using workspace: $ws"
if ($Rebuild) {
  if (Test-Path (Join-Path $ws 'build')) { Remove-Item -Recurse -Force (Join-Path $ws 'build') }
  if (Test-Path (Join-Path $ws 'install')) { Remove-Item -Recurse -Force (Join-Path $ws 'install') }
  if (Test-Path (Join-Path $ws 'log')) { Remove-Item -Recurse -Force (Join-Path $ws 'log') }
}
# Build
Push-Location $ws
colcon build --symlink-install
Pop-Location
Write-Host "Build complete. To source: .\\ros_ws\\install\\local_setup.ps1"
