# 환경 설정 가이드 (WSL2 + Ubuntu 22.04)

## 1. WSL2 설치 (Windows 11)

### 1.1 WSL2 활성화
PowerShell을 **관리자 권한**으로 실행하고 다음 명령어 실행:

```powershell
# WSL 설치
wsl --install

# Ubuntu 22.04 설치 (만약 기본 버전이 아니라면)
wsl --install -d Ubuntu-22.04
```

### 1.2 WSL2로 업그레이드 (필요시)
```powershell
# 기본 버전 확인
wsl --list --verbose

# WSL2로 설정
wsl --set-default-version 2
wsl --set-version Ubuntu-22.04 2
```

### 1.3 Ubuntu 초기 설정
WSL2 터미널에서:
```bash
# 패키지 업데이트
sudo apt update && sudo apt upgrade -y

# 필수 빌드 도구 설치
sudo apt install -y build-essential python3.10 python3.10-venv python3-pip git

# Python 3.10 확인
python3.10 --version
```

## 2. Python 가상환경 설정

### 2.1 프로젝트 디렉토리로 이동
```bash
cd /mnt/c/Users/lab/dwa-constraint-accident-analysis
```

### 2.2 가상환경 생성 및 활성화 (WSL 파일시스템 권장)
Windows 경로(`/mnt/c/...`)에 venv를 만들면 권한 문제(activate 파일 생성 실패)가 발생할 수 있습니다.  
따라서 **WSL 리눅스 홈 디렉토리**에 venv를 만드는 것을 권장합니다.

```bash
# venv 저장용 디렉토리 생성
mkdir -p ~/venvs

# 가상환경 생성 (WSL 홈에 생성)
python3.10 -m venv ~/venvs/dasc

# 활성화
source ~/venvs/dasc/bin/activate

# pip 업그레이드
pip install --upgrade pip setuptools wheel
```

## 3. 의존성 설치

### 3.1 기본 설치
```bash
pip install -r requirements.txt
```

### 3.2 일반적인 설치 오류 및 해결

#### 오류 1: PyBullet 빌드 실패
**증상**: `error: Microsoft Visual C++ 14.0 is required`

**해결**:
```bash
# WSL2에서는 Visual C++가 필요 없지만, 빌드 도구가 필요할 수 있음
sudo apt install -y gcc g++ libgl1-mesa-glx libglu1-mesa libxi6 libxext6

# PyBullet 재설치
pip install --no-cache-dir pybullet
```

#### 오류 2: gym-pybullet-drones 설치 실패
**증상**: `No matching distribution found for gym-pybullet-drones`

**해결**:
```bash
# PyPI 대신 GitHub에서 설치
pip install git+https://github.com/utiasDSL/gym-pybullet-drones.git
```

#### 오류 3: numpy 버전 호환성
**증상**: `numpy.dtype size changed`

**해결**:
```bash
# numpy 재설치
pip install --upgrade --force-reinstall numpy
```

#### 오류 4: GUI 관련 오류 (headless 모드)
**증상**: `No module named 'pygame'` 또는 `X server` 오류

**해결**:
- Headless 모드로 실행 (코드에서 자동 처리됨)
- 필요시 X11 forwarding 비활성화 확인:
```bash
export DISPLAY=:0
```

#### 오류 5: SSL 인증서 오류
**증상**: `SSL: CERTIFICATE_VERIFY_FAILED`

**해결**:
```bash
# 인증서 업데이트
sudo apt install --reinstall ca-certificates
```

## 4. 설치 검증

### 4.1 패키지 확인
```bash
python -c "import pybullet; import gymnasium; import gym_pybullet_drones; print('✓ All imports successful')"
```

### 4.2 Smoke test 실행
```bash
python scripts/smoke_test.py
```

성공하면 "Smoke test passed!" 메시지가 출력됩니다.

## 5. 추가 설정 (선택사항)

### 5.1 시각화 없이 실행 (권장)
환경 변수 설정:
```bash
export PYBULLET_EGL=0
export MUJOCO_GL=osmesa
```

### 5.2 재현성을 위한 시드 설정
코드에서 자동으로 처리되지만, 환경 변수로도 설정 가능:
```bash
export PYTHONHASHSEED=0
```

## 6. 문제 해결

### 디버깅 팁
1. **WSL2 메모리 부족**: `~/.wslconfig` 파일 생성
   ```
   [wsl2]
   memory=4GB
   ```

2. **파일 시스템 권한**: WSL2에서 Windows 파일 시스템 접근 시 느릴 수 있음. 가능하면 Linux 파일 시스템(`~/projects`) 사용 권장.

3. **Python 경로 문제**: 가상환경이 활성화되지 않은 경우:
   ```bash
   which python  # 가상환경 경로 확인
   ```
