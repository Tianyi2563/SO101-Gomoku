$ErrorActionPreference = 'Stop'
$taskRoot = 'C:\Users\Lucifer\Desktop\SO101五子棋项目'
try {
    wsl.exe --install -d Ubuntu --no-launch *> "$taskRoot\data\wsl-admin-install.log"
    $LASTEXITCODE | Set-Content "$taskRoot\data\wsl-admin-install.exit"
} catch {
    $_ | Out-File "$taskRoot\data\wsl-admin-install-error.log"
}
