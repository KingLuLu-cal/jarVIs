# Set your COM port and path to handle.exe
$comPort = "COM3"
$handlePath = "C:\Tools\handle.exe"

# Run handle.exe to find the process using COM3
$handleOutput = & "$handlePath" -a $comPort 2>&1

# Look for PID in the output
if ($handleOutput -match "pid: (\d+)") {
    $pid = $matches[1]
    Write-Host "COM3 is being used by PID: $pid. Killing it..."
    
    # Kill the process
    Stop-Process -Id $pid -Force
    Write-Host "Process $pid killed."
} else {
    Write-Host "No process is currently using $comPort."
}
