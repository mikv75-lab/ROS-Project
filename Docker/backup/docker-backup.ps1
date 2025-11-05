param(
    [Parameter(Mandatory = $true)][ValidateSet("backup","restore")]$Action,
    [Parameter(Mandatory = $false)]$ContainerName = "moveit",
    [Parameter(Mandatory = $false)]$VolumeName = "root",
    [Parameter(Mandatory = $false)]$BackupDir = ".\backup",
    [Parameter(Mandatory = $false)]$BackupFile # für Restore optional übergeben
)

# Ordner vorbereiten
if (!(Test-Path $BackupDir)) {
    New-Item -ItemType Directory -Path $BackupDir | Out-Null
}

$timestamp = Get-Date -Format "yyyyMMdd-HHmmss"

if ($Action -eq "backup") {
    Write-Host "=== BACKUP startet ===" -ForegroundColor Cyan

    # 1. Container als Image sichern (stabil ohne gzip für Windows)
    $imageName = "$ContainerName-backup:$timestamp"
    Write-Host "→ Docker Image wird erzeugt: $imageName"
    docker commit $ContainerName $imageName | Out-Null

    $imageBackupFile = "$BackupDir\$ContainerName-image-$timestamp.tar"
    Write-Host "→ Docker Image wird exportiert nach: $imageBackupFile"
    docker save -o $imageBackupFile $imageName
    Write-Host "✅ Image Backup abgeschlossen!"

    # 2. Volume sichern
    $volumeBackupFile = "$BackupDir\$VolumeName-volume-$timestamp.tar.gz"
    Write-Host "→ Volume '$VolumeName' wird gesichert nach $volumeBackupFile"
    docker run --rm -v ${VolumeName}:/${VolumeName} -v "${PWD}:/backup" alpine sh -c "tar czf /backup/${VolumeName}-volume-$timestamp.tar.gz -C / ${VolumeName}"

    Write-Host "✅ Volume Backup abgeschlossen!" -ForegroundColor Green
    Write-Host "`n----- BACKUP FILES -----"
    Write-Host "  Image:  $imageBackupFile"
    Write-Host "  Volume: $volumeBackupFile"
}

elseif ($Action -eq "restore") {
    if (-not $BackupFile) {
        Write-Host "Bitte Backupfile angeben mit -BackupFile" -ForegroundColor Yellow
        exit
    }

    Write-Host "=== RESTORE startet ===" -ForegroundColor Cyan

    if ($BackupFile -like "*image*") {
        Write-Host "→ Docker Image wird geladen aus $BackupFile"
        docker load -i $BackupFile
        Write-Host "✅ Image Restore abgeschlossen"
    }
    elseif ($BackupFile -like "*volume*") {
        Write-Host "→ Volume Restore nach '$VolumeName'"
        docker volume create $VolumeName | Out-Null
        docker run --rm -v ${VolumeName}:/${VolumeName} -v "${PWD}:/backup" alpine sh -c "cd /${VolumeName} && tar xzf /backup/$BackupFile --strip 1"
        Write-Host "✅ Volume Restore abgeschlossen"
    }
    else {
        Write-Host "BackupFile-Typ wird nicht erkannt. Muss *image* oder *volume* enthalten!" -ForegroundColor Red
    }
}
else {
    Write-Host "Ungültige Aktion. Nutze -Action backup oder restore"
}
