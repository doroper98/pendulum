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
        this.maxForce = 30.0;   // 최대 제어력 (N) - 감소
    }

    /**
     * 시스템의 미분 방정식 (간소화된 버전)
     */
    derivatives(state, force) {
        const [x, x_dot, theta, theta_dot] = state;

        const sin_theta = Math.sin(theta);
        const cos_theta = Math.cos(theta);

        // 간소화된 운동 방정식
        const total_mass = this.M + this.m;
        const pole_mass_length = this.m * this.L;

        // 분모 계산
        const temp = (force + pole_mass_length * theta_dot * theta_dot * sin_theta) / total_mass;

        // 펜듈럼 각가속도
        const theta_ddot = (this.g * sin_theta - cos_theta * temp) /
            (this.L * (4.0 / 3.0 - this.m * cos_theta * cos_theta / total_mass));

        // 카트 가속도
        const x_ddot = temp - pole_mass_length * theta_ddot * cos_theta / total_mass;

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

        // 조정된 제어 파라미터
        this.Kp = 40.0;         // 각도 비례 게인 (증가)
        this.Kd = 15.0;         // 각속도 미분 게인 (증가)
        this.Kp_cart = 2.0;     // 카트 위치 게인 (증가)
        this.Kd_cart = 8.0;     // 카트 속도 게인 (증가)

        // 스윙업 파라미터
        this.K_energy = 8.0;    // 에너지 제어 게인 (증가)

        // 모드 전환
        this.balanceThreshold = 0.35; // 라디안 (약 20도)
        this.mode = 'swing';

        // 초기 충격
        this.kickApplied = false;
        this.kickTime = 0;
    }

    /**
     * 에너지 계산
     */
    getEnergy() {
        const p = this.pendulum;
        const h = -p.L * Math.cos(p.theta);
        const v = p.L * p.theta_dot;

        const PE = p.m * p.g * h;
        const KE = 0.5 * p.m * v * v;

        return PE + KE;
    }

    /**
     * 스윙업 제어 (개선)
     */
    swingUpControl() {
        const E_target = this.pendulum.m * this.pendulum.g * this.pendulum.L;
        const E = this.getEnergy();
        const E_error = E_target - E;

        // 에너지가 부족하면 펜듈럼과 같은 방향으로 힘을 가함
        let sign = Math.sign(this.pendulum.theta_dot * Math.cos(this.pendulum.theta));

        // 각속도가 너무 작으면 방향을 강제로 설정
        if (Math.abs(this.pendulum.theta_dot) < 0.05) {
            sign = Math.cos(this.pendulum.theta) > 0 ? 1 : -1;
        }

        // 에너지 오차에 비례하는 힘
        let force = this.K_energy * E_error * sign;

        // 카트가 중앙으로 돌아오도록 (약하게)
        force -= 1.0 * this.pendulum.x;
        force -= 2.0 * this.pendulum.x_dot;

        return force;
    }

    /**
     * 밸런싱 제어 (단순 PD)
     */
    balanceControl() {
        // 각도 오차 (목표: 0)
        const theta_error = -this.pendulum.theta;

        // PD 제어
        const force_angle = this.Kp * theta_error + this.Kd * (-this.pendulum.theta_dot);

        // 카트 위치 제어
        const force_cart = -this.Kp_cart * this.pendulum.x - this.Kd_cart * this.pendulum.x_dot;

        return force_angle + force_cart;
    }

    /**
     * 제어 업데이트
     */
    update(dt) {
        if (!this.pendulum.controlActive) {
            this.pendulum.F = 0;
            this.mode = 'swing';
            this.kickApplied = false;
            this.kickTime = 0;
            return;
        }

        // 초기 충격 (0.4초 동안, 더 강하게)
        if (!this.kickApplied) {
            if (this.kickTime === 0) {
                this.kickTime = performance.now();
            }

            const elapsed = (performance.now() - this.kickTime) / 1000;
            if (elapsed < 0.4) {
                // 더 강한 진동
                this.pendulum.F = 20.0 * Math.sin(elapsed * 12);
                return;
            } else {
                this.kickApplied = true;
            }
        }

        // 모드 결정
        const angleFromTop = Math.abs(this.pendulum.theta);

        if (angleFromTop < this.balanceThreshold && Math.abs(this.pendulum.theta_dot) < 1.5) {
            this.mode = 'balance';
        } else {
            this.mode = 'swing';
        }

        // 제어력 계산
        let force;
        if (this.mode === 'balance') {
            force = this.balanceControl();
        } else {
            force = this.swingUpControl();
        }

        // 제어력 제한
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
