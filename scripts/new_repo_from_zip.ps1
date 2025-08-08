param(
  [Parameter(Mandatory = $true)][string]$ZipPath,
  [string]$DestRoot,
  [switch]$NoGit
)
$ErrorActionPreference = 'Stop'

if (-not (Test-Path -LiteralPath $ZipPath)) { throw "Zip not found: $ZipPath" }
if (-not $DestRoot) { $DestRoot = (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)) }
if (-not (Test-Path -LiteralPath $DestRoot)) { throw "Destination root not found: $DestRoot" }

$base = [IO.Path]::GetFileNameWithoutExtension($ZipPath)
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$tmp = Join-Path $DestRoot ("${base}_extract_${stamp}")
New-Item -ItemType Directory -Force -Path $tmp | Out-Null
Expand-Archive -LiteralPath $ZipPath -DestinationPath $tmp -Force

# Determine content root (handle single top-level dir)
$children = Get-ChildItem -LiteralPath $tmp
if ($children.Count -eq 1 -and $children[0].PSIsContainer) {
  $contentRoot = $children[0].FullName
} else {
  $contentRoot = $tmp
}

# Choose final target folder (avoid collision)
$target = Join-Path $DestRoot $base
if (Test-Path -LiteralPath $target) { $target = Join-Path $DestRoot ("${base}_${stamp}") }
New-Item -ItemType Directory -Force -Path $target | Out-Null

# Move content to final target
Get-ChildItem -LiteralPath $contentRoot | ForEach-Object {
  Move-Item -LiteralPath $_.FullName -Destination $target -Force
}

# Cleanup temp
Remove-Item -Recurse -Force $tmp

# Initialize a new Git repo unless disabled
if (-not $NoGit) {
  Push-Location $target
  git init | Out-Null
  # Set main branch if possible
  try { git symbolic-ref --quiet HEAD 2>$null | Out-Null } catch {}
  $hasHead = git rev-parse --verify HEAD 2>$null
  if (-not $hasHead) {
    try { git checkout -b main | Out-Null } catch {}
  }
  git add -A
  git commit -m ("chore: initial import from '" + (Split-Path -Leaf $ZipPath) + "'") | Out-Null
  Pop-Location
}

Write-Host "New repo folder: $target"
if (-not $NoGit) { Write-Host "Git repo initialized and initial commit created." }
