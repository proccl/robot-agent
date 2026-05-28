param(
    [string]$HostAddr = "127.0.0.1",
    [int]$Port = 12345,
    [string]$Message = '{"cmd":"get_status"}'
)

try {
    $client = New-Object System.Net.Sockets.TcpClient($HostAddr, $Port)
    $stream = $client.GetStream()
    $client.ReceiveTimeout = 5000
    
    # Send message with LF terminator
    $msg = $Message.Trim("'").Trim('"')
    $bytes = [System.Text.Encoding]::UTF8.GetBytes($msg + "`n")
    $stream.Write($bytes, 0, $bytes.Length)
    
    # Read response (blocking with timeout)
    $buffer = New-Object byte[] 4096
    $bytesRead = $stream.Read($buffer, 0, $buffer.Length)
    
    if ($bytesRead -gt 0) {
        $response = [System.Text.Encoding]::UTF8.GetString($buffer, 0, $bytesRead).Trim()
        Write-Output $response
    } else {
        Write-Error "No response received"
        exit 1
    }
    
    $stream.Close()
    $client.Close()
} catch {
    Write-Error "TCP error: $_"
    exit 1
}
