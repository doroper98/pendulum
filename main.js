/**
 * 메인 애플리케이션
 */

// 전역 변수
let pendulum;
let controller;
let renderer;
let animationId = null;
let lastTime = 0;

// DOM 요소
const canvas = document.getElementById('simulationCanvas');
const startBtn = document.getElementById('startBtn');
const stopBtn = document.getElementById('stopBtn');
const angleValue = document.getElementById('angleValue');
const positionValue = document.getElementById('positionValue');
const modeValue = document.getElementById('modeValue');
const forceValue = document.getElementById('forceValue');

/**
 * 초기화
 */
function init() {
    pendulum = new InvertedPendulum();
    controller = new PendulumController(pendulum);
    renderer = new Renderer(canvas, pendulum);

    // 마우스 드래그 활성화
    renderer.enableMouseDrag();

    // 초기 렌더링
    renderer.render();
    updateTelemetry();

    // 이벤트 리스너
    startBtn.addEventListener('click', startSimulation);
    stopBtn.addEventListener('click', stopSimulation);
}

/**
 * 시뮬레이션 시작
 */
function startSimulation() {
    pendulum.controlActive = true;

    // 버튼 상태 업데이트
    startBtn.disabled = true;
    stopBtn.disabled = false;

    // 애니메이션 시작
    if (!animationId) {
        lastTime = performance.now();
        animationId = requestAnimationFrame(animate);
    }

    // UI 효과
    document.body.classList.add('active-simulation');
}

/**
 * 시뮬레이션 종료
 */
function stopSimulation() {
    pendulum.controlActive = false;

    // 버튼 상태 업데이트
    startBtn.disabled = false;
    stopBtn.disabled = true;

    // UI 효과
    document.body.classList.remove('active-simulation');
}

/**
 * 애니메이션 루프
 */
function animate(currentTime) {
    const dt = Math.min((currentTime - lastTime) / 1000, 0.05); // 최대 50ms
    lastTime = currentTime;

    if (dt > 0) {
        // 제어 업데이트
        controller.update(dt);

        // 물리 시뮬레이션 업데이트
        pendulum.update(dt);

        // 렌더링
        renderer.render();

        // 텔레메트리 업데이트
        updateTelemetry();
    }

    animationId = requestAnimationFrame(animate);
}

/**
 * 텔레메트리 업데이트
 */
function updateTelemetry() {
    // 각도 (도 단위로 변환, 0 = 위)
    const angleDeg = (pendulum.theta * 180 / Math.PI).toFixed(1);
    angleValue.textContent = `${angleDeg}°`;

    // 각도에 따라 색상 변경
    const angleFromTop = Math.abs(pendulum.theta);
    if (angleFromTop < 0.3) {
        angleValue.style.color = '#10b981'; // 녹색 (균형 상태)
    } else if (angleFromTop < 1.0) {
        angleValue.style.color = '#f59e0b'; // 주황색 (중간)
    } else {
        angleValue.style.color = '#6366f1'; // 보라색 (아래)
    }

    // 위치
    positionValue.textContent = `${pendulum.x.toFixed(2)} m`;

    // 제어 모드
    modeValue.textContent = controller.getMode();

    // 모드에 따라 색상 변경
    const mode = controller.getMode();
    if (mode === '밸런싱') {
        modeValue.style.color = '#10b981';
    } else if (mode === '스윙업') {
        modeValue.style.color = '#f59e0b';
    } else {
        modeValue.style.color = '#6366f1';
    }

    // 제어력
    forceValue.textContent = `${pendulum.F.toFixed(1)} N`;

    // 제어력 방향에 따라 색상 변경
    if (Math.abs(pendulum.F) < 0.1) {
        forceValue.style.color = '#6366f1';
    } else if (pendulum.F > 0) {
        forceValue.style.color = '#10b981';
    } else {
        forceValue.style.color = '#ef4444';
    }
}

/**
 * 페이지 로드 시 초기화
 */
window.addEventListener('load', init);

/**
 * 페이지 언로드 시 정리
 */
window.addEventListener('beforeunload', () => {
    if (animationId) {
        cancelAnimationFrame(animationId);
    }
});
