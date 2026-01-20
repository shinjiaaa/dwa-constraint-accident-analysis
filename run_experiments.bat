@echo off
REM Windows 배치 파일: S1과 S2 실험 각 10회 실행

echo === Starting Experiments ===

REM S1 실험 10회
echo.
echo === Running S1 experiments (10 runs) ===
for /L %%i in (0,1,9) do (
    echo Running S1 seed %%i...
    wsl bash -c "cd /mnt/c/Users/lab/dwa-constraint-accident-analysis && source ~/venvs/dasc/bin/activate && python scripts/run_scenario.py --scenario s1 --seed %%i --out outputs/ --gui"
)

REM S2 실험 10회
echo.
echo === Running S2 experiments (10 runs) ===
for /L %%i in (0,1,9) do (
    echo Running S2 seed %%i...
    wsl bash -c "cd /mnt/c/Users/lab/dwa-constraint-accident-analysis && source ~/venvs/dasc/bin/activate && python scripts/run_scenario.py --scenario s2 --seed %%i --out outputs/ --gui"
)

REM 결과 요약 생성
echo.
echo === Generating paper results ===
wsl bash -c "cd /mnt/c/Users/lab/dwa-constraint-accident-analysis && source ~/venvs/dasc/bin/activate && python scripts/generate_paper_results.py --outputs outputs --output outputs/experimental_results_summary.md"

echo.
echo === All experiments completed! (Total: 20 runs) ===
pause
