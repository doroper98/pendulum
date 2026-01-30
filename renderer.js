/**
 * Canvas 렌더링 클래스
 */

class Renderer {
    constructor(canvas, pendulum) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.pendulum = pendulum;

        // Canvas 크기 설정
        this.resizeCanvas();
        window.addEventListener('resize', () => this.resizeCanvas());

        // 렌더링 스케일
        this.scale = 80; // 픽셀 per 미터

        // 색상
        this.colors = {
            rail: '#475569',
            cart: '#6366f1',
            cartGlow: 'rgba(99, 102, 241, 0.5)',
            pendulum: '#8b5cf6',
            pendulumGlow: 'rgba(139, 92, 246, 0.5)',
            joint: '#f59e0b',
            background: 'rgba(10, 14, 39, 0.5)',
            grid: 'rgba(99, 102, 241, 0.1)'
        };
    }

    resizeCanvas() {
        const rect = this.canvas.getBoundingClientRect();
        this.canvas.width = rect.width;
        this.canvas.height = rect.height;
        this.centerX = this.canvas.width / 2;
        this.centerY = this.canvas.height / 2 - 50;
    }

    clear() {
        this.ctx.fillStyle = this.colors.background;
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    }

    drawGrid() {
        this.ctx.strokeStyle = this.colors.grid;
        this.ctx.lineWidth = 1;

        // 수직선
        for (let x = this.centerX % 40; x < this.canvas.width; x += 40) {
            this.ctx.beginPath();
            this.ctx.moveTo(x, 0);
            this.ctx.lineTo(x, this.canvas.height);
            this.ctx.stroke();
        }

        // 수평선
        for (let y = 0; y < this.canvas.height; y += 40) {
            this.ctx.beginPath();
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(this.canvas.width, y);
            this.ctx.stroke();
        }
    }

    drawRail() {
        const railY = this.centerY;
        const railLength = this.pendulum.maxX * 2 * this.scale;

        this.ctx.strokeStyle = this.colors.rail;
        this.ctx.lineWidth = 4;
        this.ctx.lineCap = 'round';

        this.ctx.beginPath();
        this.ctx.moveTo(this.centerX - railLength / 2, railY);
        this.ctx.lineTo(this.centerX + railLength / 2, railY);
        this.ctx.stroke();

        // 레일 끝 마커
        this.ctx.fillStyle = this.colors.rail;
        const markerSize = 8;
        this.ctx.fillRect(this.centerX - railLength / 2 - markerSize / 2, railY - 10, markerSize, 20);
        this.ctx.fillRect(this.centerX + railLength / 2 - markerSize / 2, railY - 10, markerSize, 20);
    }

    drawCart() {
        const cartX = this.centerX + this.pendulum.x * this.scale;
        const cartY = this.centerY;
        const cartWidth = 60;
        const cartHeight = 40;

        // 그림자/글로우
        this.ctx.shadowColor = this.colors.cartGlow;
        this.ctx.shadowBlur = 20;

        // 카트 본체
        this.ctx.fillStyle = this.colors.cart;
        this.ctx.fillRect(
            cartX - cartWidth / 2,
            cartY - cartHeight / 2,
            cartWidth,
            cartHeight
        );

        this.ctx.shadowBlur = 0;

        // 카트 테두리
        this.ctx.strokeStyle = '#818cf8';
        this.ctx.lineWidth = 2;
        this.ctx.strokeRect(
            cartX - cartWidth / 2,
            cartY - cartHeight / 2,
            cartWidth,
            cartHeight
        );

        // 바퀴
        const wheelRadius = 6;
        const wheelY = cartY + cartHeight / 2;

        this.ctx.fillStyle = '#1e293b';
        this.ctx.beginPath();
        this.ctx.arc(cartX - 20, wheelY, wheelRadius, 0, Math.PI * 2);
        this.ctx.fill();
        this.ctx.beginPath();
        this.ctx.arc(cartX + 20, wheelY, wheelRadius, 0, Math.PI * 2);
        this.ctx.fill();

        this.ctx.strokeStyle = '#475569';
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.arc(cartX - 20, wheelY, wheelRadius, 0, Math.PI * 2);
        this.ctx.stroke();
        this.ctx.beginPath();
        this.ctx.arc(cartX + 20, wheelY, wheelRadius, 0, Math.PI * 2);
        this.ctx.stroke();

        return { x: cartX, y: cartY };
    }

    drawPendulum(cartPos) {
        const pendulumLength = this.pendulum.L * this.scale;
        const endX = cartPos.x + pendulumLength * Math.sin(this.pendulum.theta);
        const endY = cartPos.y + pendulumLength * Math.cos(this.pendulum.theta);

        // 펜듈럼 막대
        this.ctx.strokeStyle = this.colors.pendulum;
        this.ctx.lineWidth = 6;
        this.ctx.lineCap = 'round';

        this.ctx.shadowColor = this.colors.pendulumGlow;
        this.ctx.shadowBlur = 15;

        this.ctx.beginPath();
        this.ctx.moveTo(cartPos.x, cartPos.y);
        this.ctx.lineTo(endX, endY);
        this.ctx.stroke();

        this.ctx.shadowBlur = 0;

        // 조인트 (회전축)
        this.ctx.fillStyle = this.colors.joint;
        this.ctx.beginPath();
        this.ctx.arc(cartPos.x, cartPos.y, 8, 0, Math.PI * 2);
        this.ctx.fill();

        this.ctx.strokeStyle = '#fbbf24';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();

        // 펜듈럼 끝 질량
        this.ctx.fillStyle = this.colors.pendulum;
        this.ctx.shadowColor = this.colors.pendulumGlow;
        this.ctx.shadowBlur = 20;

        this.ctx.beginPath();
        this.ctx.arc(endX, endY, 12, 0, Math.PI * 2);
        this.ctx.fill();

        this.ctx.shadowBlur = 0;

        this.ctx.strokeStyle = '#a78bfa';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();
    }

    drawForceIndicator(cartPos) {
        if (!this.pendulum.controlActive || Math.abs(this.pendulum.F) < 0.1) return;

        const forceScale = 2; // 시각화 스케일
        const forceLength = this.pendulum.F * forceScale;
        const arrowY = cartPos.y - 40;

        // 화살표
        this.ctx.strokeStyle = this.pendulum.F > 0 ? '#10b981' : '#ef4444';
        this.ctx.fillStyle = this.pendulum.F > 0 ? '#10b981' : '#ef4444';
        this.ctx.lineWidth = 3;

        this.ctx.beginPath();
        this.ctx.moveTo(cartPos.x, arrowY);
        this.ctx.lineTo(cartPos.x + forceLength, arrowY);
        this.ctx.stroke();

        // 화살표 머리
        const headSize = 8;
        const direction = Math.sign(forceLength);
        this.ctx.beginPath();
        this.ctx.moveTo(cartPos.x + forceLength, arrowY);
        this.ctx.lineTo(cartPos.x + forceLength - direction * headSize, arrowY - headSize / 2);
        this.ctx.lineTo(cartPos.x + forceLength - direction * headSize, arrowY + headSize / 2);
        this.ctx.closePath();
        this.ctx.fill();
    }

    render() {
        this.clear();
        this.drawGrid();
        this.drawRail();
        const cartPos = this.drawCart();
        this.drawPendulum(cartPos);
        this.drawForceIndicator(cartPos);
    }
}
