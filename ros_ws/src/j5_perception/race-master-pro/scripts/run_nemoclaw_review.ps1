param(
    [Parameter(Mandatory = $true)][string]$RaceId,
    [string]$Pi = 'http://192.168.1.134:8080',
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
$openshell = '/home/casua/.local/bin/openshell'
$sandboxReview = "/sandbox/.openclaw/workspace/j5-review-run/$RaceId"
$sandboxPackage = "/sandbox/.openclaw/workspace/j5-review-$RaceId.tar.gz"
$package = Join-Path $reviewDir 'review-package.tar.gz'

if (-not (Test-Path -LiteralPath $python)) { throw "Python venv not found: $python" }
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
& wsl.exe -e $openshell sandbox exec --name planter --timeout 30 -- mkdir -p $sandboxReview
if ($LASTEXITCODE -ne 0) { throw 'Could not create NemoClaw review workspace' }
& wsl.exe -e $openshell sandbox upload planter $wslPackage $sandboxPackage
if ($LASTEXITCODE -ne 0) { throw 'Could not upload review package to NemoClaw' }
& wsl.exe -e $openshell sandbox exec --name planter --timeout 120 -- tar -xzf $sandboxPackage -C $sandboxReview
if ($LASTEXITCODE -ne 0) { throw 'Could not unpack review package in NemoClaw' }
& wsl.exe -e $openshell sandbox exec --name planter --timeout 3600 -- python3 "$sandboxReview/nemoclaw_batch_review.py" "$sandboxReview/review.json" --limit $Limit
$visionExit = $LASTEXITCODE
& wsl.exe -e $openshell sandbox download planter "$sandboxReview/classified.json" $wslClassified
if ($LASTEXITCODE -ne 0) { throw 'Could not download NemoClaw classifications; prepared strips remain on disk' }
& $python $worker --pi $Pi --race-id $RaceId --output-dir $OutputDir --import-classifications $classified --submit
if ($LASTEXITCODE -ne 0) { throw 'Could not submit the proposed result to the Pi' }
if ($visionExit -ne 0) { throw 'NemoClaw stopped early; completed classifications were saved and submitted' }
