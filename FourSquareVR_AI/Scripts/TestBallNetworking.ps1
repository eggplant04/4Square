param(
    [string]$Case = 'IPFinal',
    [int]$Port = 17779,
    [switch]$Impaired,
    [switch]$Listen,
    [switch]$DynamicHands
)
$ErrorActionPreference = 'Stop'
if ($Case -notmatch '^[A-Za-z0-9_-]+$' -or $Port -lt 1024 -or $Port -gt 65535) { throw 'Invalid test identity or port.' }
$projectRoot = Split-Path -Parent $PSScriptRoot
$editor = 'S:/Program Files/EpicGames/UE_5.6/Engine/Binaries/Win64/UnrealEditor-Cmd.exe'
$project = Join-Path $projectRoot 'FourSquareVR.uproject'
$logRoot = Join-Path $projectRoot 'Saved/Logs/BallStage5'
New-Item -ItemType Directory -Force -Path $logRoot | Out-Null
$common = @($project, '-BallNetworkIPTest', '-unattended', '-nop4', '-nosplash', '-NullRHI', '-nosound', '-nohmd', '-nosteam', '"-ExecCmds=t.MaxFPS 120"')
if ($DynamicHands) { $common += '-BallTestDynamicHands' }
function Start-BallProcess($Role, $Connection, $Mode, $Seconds) {
    $log = Join-Path $logRoot ($Case + $Role + '.log')
    $args = @($common[0], $Connection) + $common[1..($common.Count-1)] + @($Mode, "-BallNetTestSeconds=$Seconds", "-abslog=$log")
    if ($Role -eq 'Server') { $args += "-port=$Port" }
    if ($Impaired) { $args += @('-PktLag=40', '-PktLagVariance=15', '-PktLoss=10') }
    Start-Process -FilePath $editor -ArgumentList $args -WindowStyle Hidden -PassThru
}
$mode = if ($Listen) { '-game' } else { '-server' }
$server = Start-BallProcess 'Server' '/Engine/Maps/Entry?listen?game=/Script/FourSquareVR.BallNetworkTestGameMode' $mode 70
Write-Output "Server PID=$($server.Id)"
Start-Sleep -Seconds 8
$first = Start-BallProcess 'Client1' "127.0.0.1:$Port" '-game' 52
Write-Output "Client1 PID=$($first.Id)"
Start-Sleep -Seconds 12
$late = Start-BallProcess 'Client2Late' "127.0.0.1:$Port" '-game' 38
Write-Output "Late client PID=$($late.Id)"
foreach ($process in @($server,$first,$late)) { $process.WaitForExit(); Write-Output "PID=$($process.Id) Exit=$($process.ExitCode)" }
Write-Output "Logs: $logRoot/$Case*.log; verify NETSNAP CRCs and NETTEST boundaries before calling the run passed."
