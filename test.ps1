param([string]$Python = 'python')
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    & $Python -X utf8 -B -m unittest discover -s tests -v
    if ($LASTEXITCODE -ne 0) { throw 'DRK10020 offline checks failed' }
}
finally { Pop-Location }
