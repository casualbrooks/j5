param(
  [Parameter(Mandatory=$true)][string]$ZipPath
)
$ErrorActionPreference = 'Stop'

if (-not (Test-Path -LiteralPath $ZipPath)) { throw "Zip not found: $ZipPath" }
$repo = Split-Path -Parent $PSScriptRoot
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$workspace = Split-Path -Parent $repo
$extractRoot = Join-Path $workspace ("j5_import_" + $stamp)
Expand-Archive -LiteralPath $ZipPath -DestinationPath $extractRoot -Force

$children = Get-ChildItem -LiteralPath $extractRoot
if ($children.Count -eq 1 -and $children[0].PSIsContainer) { $contentRoot = $children[0].FullName } else { $contentRoot = $extractRoot }

$label = (Split-Path -Leaf $ZipPath).Replace('.zip','') + '_' + $stamp
$docsImported = Join-Path $repo (Join-Path 'docs\imported' $label)
$readmesImported = Join-Path $docsImported 'readmes'
$containersImported = Join-Path $repo (Join-Path 'containers\imported' $label)
$devcontainerTarget = Join-Path $repo '.devcontainer'
New-Item -ItemType Directory -Force -Path $docsImported,$readmesImported,$containersImported | Out-Null

# Copy docs folder non-destructively if exists
$srcDocs = Join-Path $contentRoot 'docs'
$docsCopied = 0
if (Test-Path $srcDocs) {
  Copy-Item -Path $srcDocs\* -Destination $docsImported -Recurse -Force
  $docsCopied = (Get-ChildItem -Path $srcDocs -Recurse -File | Measure-Object).Count
}

# Copy top-level README as README.imported.md (or timestamped variant)
$topReadme = Join-Path $contentRoot 'README.md'
$topReadmeCopied = $false
if (Test-Path $topReadme) {
  $dest = Join-Path $repo 'README.imported.md'
  if (-not (Test-Path $dest)) { Copy-Item -LiteralPath $topReadme -Destination $dest -Force }
  else { Copy-Item -LiteralPath $topReadme -Destination (Join-Path $repo ("README.imported_" + $stamp + '.md')) -Force }
  $topReadmeCopied = $true
}

# Copy all other README*.md files into docs/imported/<label>/readmes preserving structure
$otherReadmes = Get-ChildItem -Path $contentRoot -Recurse -Include 'README.md','README*.md' -File | Where-Object { $_.FullName -ne $topReadme }
$readmesCount = 0
foreach ($f in $otherReadmes) {
  $rel = ($f.FullName.Substring($contentRoot.Length)) -replace '^[\\/]+',''
  $destPath = Join-Path $readmesImported $rel
  New-Item -ItemType Directory -Force -Path (Split-Path -Parent $destPath) | Out-Null
  Copy-Item -LiteralPath $f.FullName -Destination $destPath -Force
  $readmesCount++
}

# Containerization: .devcontainer
$srcDevcontainer = Join-Path $contentRoot '.devcontainer'
$devcontainerAction = 'none'
if (Test-Path $srcDevcontainer) {
  if (-not (Test-Path $devcontainerTarget)) {
    Copy-Item -Path $srcDevcontainer -Destination $repo -Recurse -Force
    $devcontainerAction = 'copied to .devcontainer'
  } else {
    $target = Join-Path $devcontainerTarget ("imported_" + $label)
    Copy-Item -Path $srcDevcontainer -Destination $target -Recurse -Force
    $devcontainerAction = "copied to $target"
  }
}

# Containerization: Docker/compose files
$containerFiles = Get-ChildItem -Path $contentRoot -Recurse -Include 'Dockerfile*','docker-compose*.yml','docker-compose*.yaml','compose*.yml','compose*.yaml','.dockerignore' -File
$containerCopied = 0
foreach ($cf in $containerFiles) {
  $rel = ($cf.FullName.Substring($contentRoot.Length)) -replace '^[\\/]+',''
  $destAtRoot = Join-Path $repo (Split-Path -Leaf $cf.FullName)
  if (-not (Test-Path $destAtRoot)) {
    Copy-Item -LiteralPath $cf.FullName -Destination $repo -Force
  } else {
    $destAlt = Join-Path $containersImported $rel
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $destAlt) | Out-Null
    Copy-Item -LiteralPath $cf.FullName -Destination $destAlt -Force
  }
  $containerCopied++
}

Write-Host "Zip: $ZipPath"
Write-Host "Extracted to: $extractRoot (content root: $contentRoot)"
Write-Host "Docs files copied: $docsCopied into $docsImported"
Write-Host "Top-level README copied: $topReadmeCopied"
Write-Host "Other READMEs copied: $readmesCount into $readmesImported"
Write-Host "Devcontainer: $devcontainerAction"
Write-Host "Container files processed: $containerCopied (fallback dir: $containersImported)"
