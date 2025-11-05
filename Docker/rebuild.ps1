param(
    [Parameter(Mandatory = $true)][ValidateSet("backup","restore")]$Action,
    [Parameter(Mandatory = $false)]$ContainerName = "thesis-gpu",
    [Parameter(Mandatory = $false)]$VolumeName = "thesis_root",
    [Parameter(Mandatory = $false)]$BackupDir = ".\backup",
    [Parameter(Mandatory = $false)]$BackupFile
)

# Ordner vorbereiten
if (!(Test-Path $BackupDir)) {
    New-Item -ItemType Directory -Path $BackupDir | Out-Null
}

$timestamp = Get-Date -Format "yyyyMMdd-HHmmss"

if ($Action -eq "backup") {
    Write-Host "=== BACKUP startet ===" -ForegroundColor Cyan

    # 1. Container sichern
    $imageName = "$ContainerName-backup:$timestamp"
    $imageBackupFile = "$BackupDir\$ContainerName-image-$timestamp.tar.gz"

    Write-Host "→ Docker Image wird exportiert nach $imageBackupFile"
    docker commit $ContainerName $imageName | Out-Null
    docker save $imageName | gzip > $imageBackupFile

    # 2. Volume sichern
    $volumeBackupFile = "$BackupDir\$VolumeName-volume-$timestamp.tar.gz"
    Write-Host "→ Docker Volume '$VolumeName' wird gesichert nach $volumeBackupFile"

    docker run --rm `
        -v ${VolumeName}:/${VolumeName} `
        -v "${PWD}:/backup" `
        alpine sh -c "tar czf /backup/${volumeBackupFile} -C / ${VolumeName}"

    Write-Host "✅ Backup abgeschlossen!" -ForegroundColor Green
}

elseif ($Action -eq "restore") {
    if (-not $BackupFile) {
        Write-Host "Bitte BackupFile angeben mit -BackupFile" -ForegroundColor Yellow
        exit
    }

    Write-Host "=== RESTORE startet ===" -ForegroundColor Cyan

    if ($BackupFile -like "*image*") {
        Write-Host "→ Docker Image wird geladen..."
        gunzip -c $BackupFile | docker load
        Write-Host "✅ Image Restore abgeschlossen" -ForegroundColor Green
    }
    elseif ($BackupFile -like "*volume*") {
        Write-Host "→ Docker Volume wird wiederhergestellt..."

        docker volume create $VolumeName | Out-Null
        docker run --rm `
            -v ${VolumeName}:/${VolumeName} `
            -v "${PWD}:/backup" `
            alpine sh -c "cd /${VolumeName} && tar xzf /backup/${BackupFile} --strip 1"

        Write-Host "✅ Volume Restore abgeschlossen" -ForegroundColor Green
    }
    else {
        Write-Host "BackupFile muss *image* oder *volume* im Namen enthalten!" -ForegroundColor Red
    }
}
