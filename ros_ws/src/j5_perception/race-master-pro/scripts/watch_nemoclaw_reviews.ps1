param(
    [Parameter(Mandatory = $true)][string]$Pi,
    [Parameter(Mandatory = $true)][string]$Sandbox,
    [Parameter(Mandatory = $true)][string]$OpenShell,
    [Parameter(Mandatory = $true)][string]$NemoClaw,
    [string]$OutputDir = "$env:USERPROFILE\j5-reviews",
    [int]$PollSeconds = 30,
    [int]$BatchSize = 10
)

$ErrorActionPreference = 'Stop'
$runner = Join-Path $PSScriptRoot 'run_nemoclaw_review.ps1'
$worker = Join-Path $PSScriptRoot 'offboard_review.py'
$project = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..\..\..')).Path
$python = Join-Path $project '.venv\Scripts\python.exe'
New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
$log = Join-Path $OutputDir 'worker.log'

function Log([string]$Text) {
    Add-Content -LiteralPath $log -Value "$(Get-Date -Format o) $Text"
}

function Review-Job([string]$RaceId) {
    $reviewDir = Join-Path $OutputDir $RaceId
    $reviewJson = Join-Path $reviewDir 'review.json'
    $classified = Join-Path $reviewDir 'classified.json'
    if ((Test-Path -LiteralPath $reviewJson) -and (Test-Path -LiteralPath $classified)) {
        $prepared = Get-Content -LiteralPath $reviewJson -Raw | ConvertFrom-Json
        $labels = Get-Content -LiteralPath $classified -Raw | ConvertFrom-Json
        $known = @($labels.items | Where-Object { $_.racer_id }).Count
        if (@($labels.items).Count -ge $BatchSize -and $known -eq 0) {
            Log "${RaceId}: prior sample found no identifiable cars; returning incomplete proposal"
            & $python $worker --pi $Pi --race-id $RaceId --output-dir $OutputDir --import-classifications $classified --submit
            if ($LASTEXITCODE -ne 0) { throw 'Could not return the incomplete proposal' }
            return
        }
    }
    $status = & wsl.exe -e $OpenShell status 2>&1
    if ($LASTEXITCODE -ne 0) {
        Log "${RaceId}: recovering NemoClaw gateway"
        & wsl.exe -e $NemoClaw $Sandbox recover | Out-Null
        if ($LASTEXITCODE -ne 0) { throw 'Could not recover NemoClaw gateway' }
    }
    Log "${RaceId}: reviewing up to $BatchSize crossing clips"
    & $runner -RaceId $RaceId -Pi $Pi -Sandbox $Sandbox -OpenShell $OpenShell -Limit $BatchSize -OutputDir $OutputDir
    if ($LASTEXITCODE -ne 0) { throw 'NemoClaw review command failed' }
}

Log 'Review worker started'
while ($true) {
    try {
        $jobs = Invoke-RestMethod -Uri "$($Pi.TrimEnd('/'))/api/quick/review/jobs" -TimeoutSec 15
        foreach ($job in @($jobs)) {
            if ($job.status -ne 'requested') { continue }
            try { Review-Job $job.race_id }
            catch { Log "$($job.race_id): $($_.Exception.Message)" }
        }
    } catch { Log "Poll failed: $($_.Exception.Message)" }
    Start-Sleep -Seconds $PollSeconds
}
