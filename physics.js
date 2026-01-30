/**
 * 역진자 물리 시뮬레이션 및 제어
 * 
 * 수학적 기반:
 * - Lagrangian Mechanics 기반 운동 방정식
 * - Runge-Kutta 4차 적분 (O(dt^5) 오차율)
 * - 에너지 보존 검증 기능
 * 
 * 제어 시스템:
 * - 에너지 기반 스윙업 (Åström method)
 * - LQR 밸런싱 (선형화 근처)
 */

class InvertedPendulum {
    constructor() {
        // ===== 물리 파라미터 =====
        this.M = 1.0;           // 카트 질량 (kg)
        this.m = 0.2;           // 펜듈럼 질량 (kg)
        this.l = 0.75;          // 펜듈럼 길이 - 질량 중심까지 (m)
        this.L = 1.5;           // 펜듈럼 전체 길이 (렌더링용, m)
        this.I = this.m * this.l * this.l / 3;  // 관성 모멘트 (균일 막대)
        this.g = 9.81;          // 중력 가속도 (m/s²)
        this.b = 0.1;           // 카트 마찰 계수
        this.c = 0.01;          // 펜듈럼 감쇠 계수

        // ===== 상태 변수 =====
        // 상태 벡터: [x, x_dot, theta, theta_dot]
        this.x = 0;             // 카트 위치 (m)
        this.x_dot = 0;         // 카트 속도 (m/s)
        this.theta = Math.PI;   // 펜듈럼 각도 (rad, 0=수직상방, π=수직하방)
        this.theta_dot = 0;     // 펜듈럼 각속도 (rad/s)

        // ===== 제어 변수 =====
        this.F = 0;             // 제어 입력 - 카트에 가하는 힘 (N)
        this.controlActive = false;

        // ===== 제약 조건 =====
        this.maxX = 4.0;        // 카트 최대 이동 거리 (m)
        this.maxForce = 50.0;   // 최대 제어력 (N)

        // ===== 에너지 모니터링 =====
        this.totalEnergy = 0;
        this.initialEnergy = 0;
    }

    /**
     * Lagrangian 역학에서 유도된 운동 방정식
     * 
     * 라그랑지안: L = T - V
     * 
     * 운동 방정식:
     * (M+m)ẍ + ml*θ̈*cos(θ) - ml*θ̇²*sin(θ) + b*ẋ = F
     * ml*ẍ*cos(θ) + (I + ml²)*θ̈ - mgl*sin(θ) + c*θ̇ = 0
     * 
     * 이를 ẍ, θ̈에 대해 정리
     */
    derivatives(state, F) {
        const [x, x_dot, theta, theta_dot] = state;

        const s = Math.sin(theta);
        const c = Math.cos(theta);

        // 물리 상수
        const { M, m, l, I, g } = this;
        const b = this.b;  // 카트 마찰
        const d = this.c;  // 펜듈럼 감쇠

        // 질량 행렬 요소
        const M11 = M + m;
        const M12 = m * l * c;
        const M21 = m * l * c;
        const M22 = I + m * l * l;

        // 판별식 (질량 행렬의 행렬식)
        const det = M11 * M22 - M12 * M21;

        // 우변항 (Generalized forces)
        const f1 = F - b * x_dot + m * l * theta_dot * theta_dot * s;
        const f2 = m * g * l * s - d * theta_dot;

        // 역행렬을 이용한 가속도 계산
        // [ẍ ]   [M11 M12]^(-1)   [f1]
        // [θ̈] = [M21 M22]      * [f2]
        const x_ddot = (M22 * f1 - M12 * f2) / det;
        const theta_ddot = (-M21 * f1 + M11 * f2) / det;

        return [x_dot, x_ddot, theta_dot, theta_ddot];
    }

    /**
     * Runge-Kutta 4차 적분
     * 오차율: O(dt^5)
     */
    rungeKutta4(dt) {
        const state = [this.x, this.x_dot, this.theta, this.theta_dot];
        const F = this.F;

        // k1
        const k1 = this.derivatives(state, F);

        // k2
        const state2 = state.map((s, i) => s + k1[i] * dt / 2);
        const k2 = this.derivatives(state2, F);

        // k3
        const state3 = state.map((s, i) => s + k2[i] * dt / 2);
        const k3 = this.derivatives(state3, F);

        // k4
        const state4 = state.map((s, i) => s + k3[i] * dt);
        const k4 = this.derivatives(state4, F);

        // 가중 평균
        const newState = state.map((s, i) =>
            s + (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) * dt / 6
        );

        [this.x, this.x_dot, this.theta, this.theta_dot] = newState;

        // 각도 정규화 [-π, π]
        while (this.theta > Math.PI) this.theta -= 2 * Math.PI;
        while (this.theta < -Math.PI) this.theta += 2 * Math.PI;

        // 카트 경계 조건 (탄성 충돌)
        if (Math.abs(this.x) > this.maxX) {
            this.x = Math.sign(this.x) * this.maxX;
            this.x_dot *= -0.5;
        }
    }

    /**
     * 시스템 전체 에너지 계산
     * E = T(운동 에너지) + V(위치 에너지)
     */
    calculateEnergy() {
        const { M, m, l, I, g } = this;
        const { x_dot, theta, theta_dot } = this;

        const s = Math.sin(theta);
        const c = Math.cos(theta);

        // 카트 운동 에너지
        const T_cart = 0.5 * M * x_dot * x_dot;

        // 펜듈럼 질량 중심 속도
        const v_px = x_dot + l * theta_dot * c;  // x 방향
        const v_py = l * theta_dot * s;          // y 방향

        // 펜듈럼 병진 운동 에너지
        const T_pend_trans = 0.5 * m * (v_px * v_px + v_py * v_py);

        // 펜듈럼 회전 운동 에너지
        const T_pend_rot = 0.5 * I * theta_dot * theta_dot;

        // 전체 운동 에너지
        const T = T_cart + T_pend_trans + T_pend_rot;

        // 위치 에너지 (기준: theta=π, 아래에서 0)
        // 위에서(theta=0): V = mgl
        const V = m * g * l * (1 - c);

        this.totalEnergy = T + V;
        return this.totalEnergy;
    }

    /**
     * 시뮬레이션 업데이트
     */
    update(dt) {
        // 안정성을 위한 서브스텝
        const substeps = 5;
        const subDt = dt / substeps;

        for (let i = 0; i < substeps; i++) {
            this.rungeKutta4(subDt);
        }

        // 에너지 계산
        this.calculateEnergy();
    }

    /**
     * 상태 리셋
     */
    reset() {
        this.x = 0;
        this.x_dot = 0;
        this.theta = Math.PI;  // 아래로 매달린 상태
        this.theta_dot = 0;
        this.F = 0;
        this.controlActive = false;
        this.calculateEnergy();
        this.initialEnergy = this.totalEnergy;
    }
}


/**
 * 2단계 제어기
 * 
 * 1. 스윙업: 에너지 기반 제어 (Åström method)
 * 2. 밸런싱: LQR 제어 (선형화 기반)
 */
class PendulumController {
    constructor(pendulum) {
        this.p = pendulum;

        // ===== 스윙업 파라미터 =====
        this.swingGain = 40.0;      // 에너지 펌핑 게인

        // ===== LQR 게인 =====
        // u = -K * [x, x_dot, theta, theta_dot]
        // 시스템 파라미터에 맞게 조정됨
        this.K = [1.0, 2.0, -40.0, -8.0];

        // ===== 모드 전환 =====
        this.switchAngle = 0.35;    // 약 20도
        this.switchVel = 2.5;       // rad/s

        // ===== 상태 =====
        this.mode = 'idle';
        this.startTime = 0;
    }

    /**
     * 스윙업 목표 에너지 계산
     * 목표: theta=0에서 정지 (수직 상방)
     */
    getTargetEnergy() {
        const { m, l, g } = this.p;
        // 수직 상방에서의 위치 에너지
        return 2 * m * g * l;
    }

    /**
     * 현재 펜듈럼 에너지 계산
     */
    getPendulumEnergy() {
        const { m, l, g, theta, theta_dot } = this.p;

        // 위치 에너지 (아래=0, 위=2mgl)
        const V = m * g * l * (1 - Math.cos(theta));

        // 운동 에너지
        const T = 0.5 * m * l * l * theta_dot * theta_dot;

        return T + V;
    }

    /**
     * 스윙업 제어 (에너지 기반)
     * 
     * Åström method:
     * u = k * (E - E_target) * sign(θ̇ * cos(θ))
     */
    swingUpControl() {
        const E = this.getPendulumEnergy();
        const E_target = this.getTargetEnergy();
        const E_error = (E - E_target) / E_target;  // 정규화된 오차

        // 에너지가 이미 충분하면 감쇠
        if (E > E_target * 1.1) {
            return -10.0 * this.p.x_dot;
        }

        // 에너지 펌핑
        const sign = Math.sign(this.p.theta_dot * Math.cos(this.p.theta));
        let u = -this.swingGain * E_error * sign;

        // 카트 위치 복원 (약하게)
        u -= 1.5 * this.p.x + 2.5 * this.p.x_dot;

        // 레일 끝 보호
        if (Math.abs(this.p.x) > this.p.maxX * 0.7) {
            u -= 25.0 * this.p.x;
        }

        return u;
    }

    /**
     * LQR 밸런싱 제어
     * 
     * 선형화된 시스템:
     * 평형점 θ ≈ 0 근처에서 sin(θ) ≈ θ, cos(θ) ≈ 1
     * 
     * 상태 피드백: u = -K * x
     */
    balanceControl() {
        const state = [
            this.p.x,
            this.p.x_dot,
            this.p.theta,
            this.p.theta_dot
        ];

        // u = -K * x
        let u = 0;
        for (let i = 0; i < 4; i++) {
            u -= this.K[i] * state[i];
        }

        // 레일 끝 보호
        if (Math.abs(this.p.x) > this.p.maxX * 0.8) {
            u -= 35.0 * this.p.x;
        }

        return u;
    }

    /**
     * 밸런싱 가능 여부 판단
     */
    canBalance() {
        const angleFromTop = Math.abs(this.p.theta);
        return angleFromTop < this.switchAngle &&
            Math.abs(this.p.theta_dot) < this.switchVel;
    }

    /**
     * 제어 업데이트
     */
    update(dt) {
        if (!this.p.controlActive) {
            this.p.F = 0;
            this.mode = 'idle';
            this.startTime = 0;
            return;
        }

        // 시작 시간 기록
        if (this.startTime === 0) {
            this.startTime = performance.now();
        }

        const elapsed = (performance.now() - this.startTime) / 1000;

        // 처음 0.4초: 강력한 초기 충격
        if (elapsed < 0.4) {
            this.p.F = 35 * Math.sin(elapsed * 18);
            this.mode = 'kick';
            return;
        }

        // 모드 결정
        if (this.canBalance()) {
            this.mode = 'balance';
        } else if (this.mode === 'balance' && Math.abs(this.p.theta) > this.switchAngle * 2) {
            this.mode = 'swing';
        } else if (this.mode !== 'balance') {
            this.mode = 'swing';
        }

        // 제어력 계산
        let u;
        if (this.mode === 'balance') {
            u = this.balanceControl();
        } else {
            u = this.swingUpControl();
        }

        // 제어력 제한
        u = Math.max(-this.p.maxForce, Math.min(this.p.maxForce, u));

        this.p.F = u;
    }

    /**
     * 현재 모드 반환
     */
    getMode() {
        switch (this.mode) {
            case 'idle': return '대기';
            case 'kick': return '초기화';
            case 'swing': return '스윙업';
            case 'balance': return 'LQR 밸런싱';
            default: return this.mode;
        }
    }
}
