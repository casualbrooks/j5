param(
    [Parameter(Mandatory = $true)][string]$RaceId,
    [string]$Pi = $env:J5_PI_URL,
    [string]$Sandbox = $env:J5_NEMOCLAW_SANDBOX,
    [string]$OpenShell = $env:J5_OPENSHELL_PATH,
    [int]$Limit = 20,
    [string]$OutputDir = "$env:USERPROFILE\j5-reviews"
)

$ErrorActionPreference = 'Stop'
$project = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..\..\..')).Path
$python = Join-Path $project '.venv\Scripts\python.exe'
$worker = Join-Path $PSScriptRoot 'offboard_review.py'
$classifier = Join-Path $PSScriptRoot 'nemoclaw_batch_review.py'
$reviewDir = Join-Path $OutputDir $RaceId
$reviewJson = Join-Path $reviewDir 'review.json'
$strips = Join-Path $reviewDir 'strips'
$classified = Join-Path $reviewDir 'classified.json'
$sandboxReview = "/sandbox/.openclaw/workspace/j5-review-run/$RaceId"
$sandboxPackage = "/sandbox/.openclaw/workspace/j5-review-$RaceId.tar.gz"
$package = Join-Path $reviewDir 'review-package.tar.gz'

if (-not (Test-Path -LiteralPath $python)) { throw "Python venv not found: $python" }
if (-not $Pi) { throw 'Pass -Pi or set J5_PI_URL on this computer' }
if (-not $Sandbox) { throw 'Pass -Sandbox or set J5_NEMOCLAW_SANDBOX on this computer' }
if (-not $OpenShell) { throw 'Pass -OpenShell or set J5_OPENSHELL_PATH on this computer' }
& $python $worker --pi $Pi --race-id $RaceId --output-dir $OutputDir --detect-only
if ($LASTEXITCODE -ne 0) { throw 'Video preparation failed' }

$localClassifier = Join-Path $reviewDir 'nemoclaw_batch_review.py'
Copy-Item -LiteralPath $classifier -Destination $localClassifier -Force
$packageItems = @('review.json', 'strips', 'nemoclaw_batch_review.py')
if (Test-Path -LiteralPath $classified) { $packageItems += 'classified.json' }
& tar.exe -czf $package -C $reviewDir @packageItems
if ($LASTEXITCODE -ne 0) { throw 'Could not package review data' }
$wslPackage = (& wsl.exe -e wslpath -u $package).Trim()
$wslClassified = (& wsl.exe -e wslpath -u $classified).Trim()
& wsl.exe -e $OpenShell sandbox exec --name $Sandbox --timeout 30 -- mkdir -p $sandboxReview
if ($LASTEXITCODE -ne 0) { throw 'Could not create NemoClaw review workspace' }
& wsl.exe -e $OpenShell sandbox upload $Sandbox $wslPackage $sandboxPackage
if ($LASTEXITCODE -ne 0) { throw 'Could not upload review package to NemoClaw' }
& wsl.exe -e $OpenShell sandbox exec --name $Sandbox --timeout 120 -- tar -xzf $sandboxPackage -C $sandboxReview
if ($LASTEXITCODE -ne 0) { throw 'Could not unpack review package in NemoClaw' }
& wsl.exe -e $OpenShell sandbox exec --name $Sandbox --timeout 3600 -- python3 "$sandboxReview/nemoclaw_batch_review.py" "$sandboxReview/review.json" --limit $Limit
$visionExit = $LASTEXITCODE
& wsl.exe -e $OpenShell sandbox download $Sandbox "$sandboxReview/classified.json" $wslClassified
if ($LASTEXITCODE -ne 0) { throw 'Could not download NemoClaw classifications; prepared strips remain on disk' }
& $python $worker --pi $Pi --race-id $RaceId --output-dir $OutputDir --import-classifications $classified --submit
if ($LASTEXITCODE -ne 0) { throw 'Could not submit the proposed result to the Pi' }
if ($visionExit -ne 0) { throw 'NemoClaw stopped early; completed classifications were saved and submitted' }
