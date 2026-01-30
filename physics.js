/**
 * 역진자 물리 시뮬레이션 및 제어 클래스
 */

class InvertedPendulum {
    constructor() {
        // 물리 파라미터
        this.M = 1.0;           // 카트 질량 (kg)
        this.m = 0.3;           // 펜듈럼 질량 (kg)
        this.L = 1.0;           // 펜듈럼 길이 (m)
        this.g = 9.81;          // 중력 가속도 (m/s^2)
        this.b = 0.1;           // 카트 마찰 계수
        this.I = this.m * this.L * this.L / 3; // 펜듈럼 관성 모멘트

        // 상태 변수
        this.x = 0;             // 카트 위치 (m)
        this.x_dot = 0;         // 카트 속도 (m/s)
        this.theta = Math.PI;   // 펜듈럼 각도 (rad, 0 = 위, PI = 아래)
        this.theta_dot = 0;     // 펜듈럼 각속도 (rad/s)

        // 제어 변수
        this.F = 0;             // 카트에 가해지는 힘 (N)
        this.controlActive = false;

        // 제약 조건
        this.maxX = 3.0;        // 카트 최대 이동 거리 (m)
        this.maxForce = 50.0;   // 최대 제어력 (N)
    }

    /**
     * 시스템의 미분 방정식 (상태 공간 방정식)
     */
    derivatives(state, force) {
        const [x, x_dot, theta, theta_dot] = state;

        const sin_theta = Math.sin(theta);
        const cos_theta = Math.cos(theta);

        // 운동 방정식 (Lagrangian 역학에서 유도)
        const denominator = this.M + this.m * sin_theta * sin_theta;

        // 카트 가속도
        const x_ddot = (force + this.m * this.L * theta_dot * theta_dot * sin_theta
            - this.b * x_dot - this.m * this.g * sin_theta * cos_theta) / denominator;

        // 펜듈럼 각가속도
        const theta_ddot = (-force * cos_theta - this.m * this.L * theta_dot * theta_dot * sin_theta * cos_theta
            + this.b * x_dot * cos_theta + (this.M + this.m) * this.g * sin_theta)
            / (this.L * denominator);

        return [x_dot, x_ddot, theta_dot, theta_ddot];
    }

    /**
     * Runge-Kutta 4차 적분법
     */
    rungeKutta4(dt) {
        const state = [this.x, this.x_dot, this.theta, this.theta_dot];

        const k1 = this.derivatives(state, this.F);

        const state2 = state.map((s, i) => s + k1[i] * dt / 2);
        const k2 = this.derivatives(state2, this.F);

        const state3 = state.map((s, i) => s + k2[i] * dt / 2);
        const k3 = this.derivatives(state3, this.F);

        const state4 = state.map((s, i) => s + k3[i] * dt);
        const k4 = this.derivatives(state4, this.F);

        // 가중 평균으로 다음 상태 계산
        const newState = state.map((s, i) =>
            s + (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) * dt / 6
        );

        [this.x, this.x_dot, this.theta, this.theta_dot] = newState;

        // 각도를 [-PI, PI] 범위로 정규화
        this.theta = ((this.theta + Math.PI) % (2 * Math.PI)) - Math.PI;

        // 카트 위치 제약
        if (Math.abs(this.x) > this.maxX) {
            this.x = Math.sign(this.x) * this.maxX;
            this.x_dot = 0;
        }
    }

    /**
     * 시뮬레이션 업데이트
     */
    update(dt) {
        // 서브스텝으로 정확도 향상
        const substeps = 4;
        const subDt = dt / substeps;

        for (let i = 0; i < substeps; i++) {
            this.rungeKutta4(subDt);
        }
    }

    /**
     * 시뮬레이션 리셋
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


class PendulumController {
    constructor(pendulum) {
        this.pendulum = pendulum;

        // PID 제어 파라미터 (밸런싱 모드) - 더 보수적으로 조정
        this.Kp_balance = 60.0;     // 비례 게인 (감소)
        this.Kd_balance = 25.0;     // 미분 게인 (감소)
        this.Ki_balance = 0.5;      // 적분 게인 (감소)

        // 스윙업 제어 파라미터
        this.K_swing = 5.0;         // 에너지 제어 게인 (감소)
        this.E_desired = this.pendulum.m * this.pendulum.g * this.pendulum.L; // 목표 에너지

        // 모드 전환 임계값
        this.balanceThreshold = 0.4; // 라디안 (약 23도, 더 넓게)

        // 제어 모드
        this.mode = 'swing';  // 'swing' 또는 'balance'

        // 적분 항
        this.integral = 0;
        this.prevError = 0;

        // 초기 충격 상태
        this.initialKickApplied = false;
        this.kickStartTime = 0;
    }

    /**
     * 펜듈럼의 현재 에너지 계산
     */
    getEnergy() {
        const p = this.pendulum;
        const h = -p.L * Math.cos(p.theta); // 높이 (아래 = -L, 위 = L)
        const v = p.L * p.theta_dot; // 펜듈럼 끝의 속도

        const PE = p.m * p.g * h; // 위치 에너지
        const KE = 0.5 * p.m * v * v; // 운동 에너지

        return PE + KE;
    }

    /**
     * 스윙업 제어 (에너지 기반)
     */
    swingUpControl() {
        const E = this.getEnergy();
        const E_error = this.E_desired - E;

        // 에너지 오차에 비례하여 힘 가함
        // 각속도가 너무 작으면 방향을 랜덤하게 (초기 충격)
        let direction = Math.sign(this.pendulum.theta_dot * Math.cos(this.pendulum.theta));
        if (Math.abs(this.pendulum.theta_dot) < 0.1) {
            direction = Math.cos(this.pendulum.theta) > 0 ? 1 : -1;
        }

        const force = this.K_swing * E_error * direction;

        return force;
    }

    /**
     * 밸런싱 제어 (PID)
     */
    balanceControl(dt) {
        // 목표: theta = 0 (수직 위), x = 0 (중앙)
        const theta_error = -this.pendulum.theta; // 각도 오차
        const x_error = -this.pendulum.x; // 위치 오차

        // PID 제어
        this.integral += theta_error * dt;
        const derivative = (theta_error - this.prevError) / dt;
        this.prevError = theta_error;

        // 적분 와인드업 방지 (더 강하게)
        this.integral = Math.max(-5, Math.min(5, this.integral));

        // 제어력 계산 (각도와 위치 모두 고려)
        const force = this.Kp_balance * theta_error
            + this.Kd_balance * (-this.pendulum.theta_dot)
            + this.Ki_balance * this.integral
            + 3.0 * x_error  // 위치 복원력 (감소)
            + 8.0 * (-this.pendulum.x_dot); // 위치 감쇠 (감소)

        return force;
    }

    /**
     * 제어 업데이트
     */
    update(dt) {
        if (!this.pendulum.controlActive) {
            this.pendulum.F = 0;
            this.mode = 'swing';
            this.integral = 0;
            this.prevError = 0;
            this.initialKickApplied = false;
            return;
        }

        // 초기 충격 적용 (시작 후 처음 0.5초 동안)
        if (!this.initialKickApplied) {
            if (this.kickStartTime === 0) {
                this.kickStartTime = performance.now();
            }
            const elapsed = (performance.now() - this.kickStartTime) / 1000;

            if (elapsed < 0.5) {
                // 작은 초기 충격을 가함
                this.pendulum.F = 15.0 * Math.sin(elapsed * 10); // 진동하는 힘
                return;
            } else {
                this.initialKickApplied = true;
            }
        }

        // 모드 결정
        const angleFromTop = Math.abs(this.pendulum.theta);

        if (angleFromTop < this.balanceThreshold && Math.abs(this.pendulum.theta_dot) < 1.5) {
            this.mode = 'balance';
        } else if (angleFromTop > this.balanceThreshold * 1.5) {
            this.mode = 'swing';
            this.integral = 0; // 적분 리셋
        }

        // 제어력 계산
        let force;
        if (this.mode === 'balance') {
            force = this.balanceControl(dt);
        } else {
            force = this.swingUpControl();
        }

        // 제어력 제한 (더 강하게)
        force = Math.max(-this.pendulum.maxForce, Math.min(this.pendulum.maxForce, force));

        this.pendulum.F = force;
    }

    /**
     * 현재 제어 모드 반환
     */
    getMode() {
        if (!this.pendulum.controlActive) return '대기';
        return this.mode === 'swing' ? '스윙업' : '밸런싱';
    }
}
