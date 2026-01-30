/**
 * 역진자 물리 시뮬레이션 및 제어
 * 
 * 검증된 2단계 제어 시스템:
 * 1. Åström 에너지 펌핑 스윙업
 * 2. LQR 밸런싱 제어
 */

class InvertedPendulum {
    constructor() {
        // 물리 파라미터
        this.M = 1.0;           // 카트 질량 (kg)
        this.m = 0.2;           // 펜듈럼 질량 (kg)
        this.L = 1.5;           // 펜듈럼 길이 (m)
        this.g = 9.81;          // 중력 가속도 (m/s^2)
        this.b = 0.5;           // 마찰 계수

        // 상태 변수
        this.x = 0;             // 카트 위치 (m)
        this.x_dot = 0;         // 카트 속도 (m/s)
        this.theta = Math.PI;   // 펜듈럼 각도 (rad, 0=위, PI=아래)
        this.theta_dot = 0;     // 펜듈럼 각속도 (rad/s)

        // 제어 변수
        this.F = 0;             // 제어력 (N)
        this.controlActive = false;

        // 제약 조건
        this.maxX = 4.0;        // 카트 최대 이동 거리 (m)
        this.maxForce = 40.0;   // 최대 제어력 (N)
    }

    /**
     * 운동 방정식 (상태 미분)
     */
    derivatives(state, force) {
        const [x, x_dot, theta, theta_dot] = state;

        const s = Math.sin(theta);
        const c = Math.cos(theta);

        // 역진자 동역학 방정식
        const totalMass = this.M + this.m;
        const l = this.L;
        const m = this.m;
        const g = this.g;

        // 분모
        const denom = l * (totalMass - m * c * c);

        // 펜듈럼 각가속도
        const theta_ddot = (g * totalMass * s - c * (force + m * l * theta_dot * theta_dot * s - this.b * x_dot)) / denom;

        // 카트 가속도
        const x_ddot = (force + m * l * (theta_dot * theta_dot * s - theta_ddot * c) - this.b * x_dot) / totalMass;

        return [x_dot, x_ddot, theta_dot, theta_ddot];
    }

    /**
     * Runge-Kutta 4차 적분
     */
    integrate(dt) {
        const state = [this.x, this.x_dot, this.theta, this.theta_dot];

        const k1 = this.derivatives(state, this.F);
        const k2 = this.derivatives(state.map((s, i) => s + k1[i] * dt / 2), this.F);
        const k3 = this.derivatives(state.map((s, i) => s + k2[i] * dt / 2), this.F);
        const k4 = this.derivatives(state.map((s, i) => s + k3[i] * dt), this.F);

        const newState = state.map((s, i) =>
            s + (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) * dt / 6
        );

        [this.x, this.x_dot, this.theta, this.theta_dot] = newState;

        // 각도 정규화 [-PI, PI]
        while (this.theta > Math.PI) this.theta -= 2 * Math.PI;
        while (this.theta < -Math.PI) this.theta += 2 * Math.PI;

        // 카트 위치 제한
        if (Math.abs(this.x) > this.maxX) {
            this.x = Math.sign(this.x) * this.maxX;
            this.x_dot *= -0.5; // 탄성 충돌
        }
    }

    /**
     * 시뮬레이션 업데이트
     */
    update(dt) {
        const substeps = 5;
        const subDt = dt / substeps;
        for (let i = 0; i < substeps; i++) {
            this.integrate(subDt);
        }
    }

    /**
     * 리셋
     */
    reset() {
        this.x = 0;
        this.x_dot = 0;
        this.theta = Math.PI;
        this.theta_dot = 0;
        this.F = 0;
        this.controlActive = false;
    }
}


/**
 * 2단계 제어기
 * 1. 에너지 기반 스윙업
 * 2. LQR 밸런싱
 */
class PendulumController {
    constructor(pendulum) {
        this.p = pendulum;

        // 스윙업 파라미터 (강화)
        this.swingGain = 50.0;

        // LQR 게인 [x, x_dot, theta, theta_dot]
        this.K = [1.0, 2.5, -35.0, -6.0];

        // 모드 전환 파라미터 (더 빨리 전환)
        this.switchAngle = 0.4;  // 약 23도
        this.switchVel = 2.0;

        // 상태
        this.mode = 'idle';
        this.startTime = 0;
    }

    /**
     * 펜듈럼 에너지 (정규화)
     * E = 0: 아래에서 정지
     * E = 1: 위에서 정지 (목표)
     */
    getEnergy() {
        const { m, L, g, theta, theta_dot } = this.p;

        // 위치 에너지: -mgL*cos(theta), 위에서 0, 아래에서 -2mgL
        // 운동 에너지: 0.5 * m * L^2 * theta_dot^2
        const E_pot = m * g * L * (1 - Math.cos(theta));
        const E_kin = 0.5 * m * L * L * theta_dot * theta_dot;

        // 목표 에너지로 정규화 (위에서 정지 = 2mgL)
        const E_target = 2 * m * g * L;

        return (E_pot + E_kin) / E_target;
    }

    /**
     * 스윙업 제어 (에너지 펌핑)
     */
    swingUpControl() {
        const E = this.getEnergy();
        const E_error = 1.0 - E;  // 목표와의 차이

        // 에너지가 과하면 감쇠
        if (E > 1.2) {
            return -5.0 * this.p.x_dot;
        }

        // 에너지 펌핑: 펜듈럼이 움직이는 방향으로 힘을 가함
        let u = this.swingGain * E_error * Math.sign(this.p.theta_dot * Math.cos(this.p.theta));

        // 카트 위치 복원 (약하게)
        u -= 1.0 * this.p.x + 2.0 * this.p.x_dot;

        // 레일 끝 보호
        if (Math.abs(this.p.x) > this.p.maxX * 0.7) {
            u -= 30.0 * this.p.x;
        }

        return u;
    }

    /**
     * LQR 밸런싱 제어
     */
    balanceControl() {
        // 상태 벡터
        const state = [this.p.x, this.p.x_dot, this.p.theta, this.p.theta_dot];

        // u = -K * x
        let u = 0;
        for (let i = 0; i < 4; i++) {
            u -= this.K[i] * state[i];
        }

        // 레일 끝 보호
        if (Math.abs(this.p.x) > this.p.maxX * 0.8) {
            u -= 40.0 * this.p.x;
        }

        return u;
    }

    /**
     * 밸런싱 가능 상태인지 확인
     */
    canBalance() {
        const angleFromTop = Math.abs(this.p.theta);
        const velOk = Math.abs(this.p.theta_dot) < this.switchVel;
        return angleFromTop < this.switchAngle && velOk;
    }

    /**
     * 업데이트
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

        // 처음 0.5초: 강력한 초기 충격
        const elapsed = (performance.now() - this.startTime) / 1000;
        if (elapsed < 0.5) {
            // 강력한 충격으로 바로 스윙업 시작
            this.p.F = 40 * Math.sin(elapsed * 15);
            this.mode = 'kick';
            return;
        }

        // 모드 결정
        if (this.canBalance()) {
            this.mode = 'balance';
        } else if (this.mode === 'balance' && Math.abs(this.p.theta) > this.switchAngle * 2) {
            // 밸런싱 실패하면 다시 스윙업
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
            case 'balance': return '밸런싱';
            default: return this.mode;
        }
    }
}
