# PowerShell 스크립트: S1과 S2 실험 각 10회 실행 (Headless 모드 - GUI 없음, 더 빠름)

Write-Host "=== Starting Experiments (Headless Mode) ===" -ForegroundColor Green

# 프로젝트 디렉토리로 이동
$projectDir = "/mnt/c/Users/lab/dwa-constraint-accident-analysis"

Write-Host "`n=== Running S1 experiments (10 runs) ===" -ForegroundColor Cyan
for ($seed = 0; $seed -lt 10; $seed++) {
    Write-Host "Running S1 seed $seed..." -ForegroundColor Yellow
    wsl bash -c "cd $projectDir && source ~/venvs/dasc/bin/activate && python scripts/run_scenario.py --scenario s1 --seed $seed --out outputs/ --headless"
}

Write-Host "`n=== Running S2 experiments (10 runs) ===" -ForegroundColor Cyan
for ($seed = 0; $seed -lt 10; $seed++) {
    Write-Host "Running S2 seed $seed..." -ForegroundColor Yellow
    wsl bash -c "cd $projectDir && source ~/venvs/dasc/bin/activate && python scripts/run_scenario.py --scenario s2 --seed $seed --out outputs/ --headless"
}

Write-Host "`n=== Generating paper results ===" -ForegroundColor Cyan
wsl bash -c "cd $projectDir && source ~/venvs/dasc/bin/activate && python scripts/generate_paper_results.py --outputs outputs --output outputs/experimental_results_summary.md"

Write-Host "`n=== All experiments completed! (Total: 20 runs) ===" -ForegroundColor Green
