import { createObjectStore } from "./Store";
import { clamp, normalizeDeg, toDeg, toRad } from "./Util";

export type RobotConstants = {
    width: number,
    height: number,
    speed: number,
    accel: number,
    lateralFriction: number
}

export const defaultRobotConstants: RobotConstants = {
    width: 14,
    height: 14,
    speed: 6,
    accel: 15,
    lateralFriction: 10
}

export const robotConstantsStore = createObjectStore<RobotConstants>(defaultRobotConstants);

export class Robot {
    public width: number;
    public height: number;
    public maxSpeed: number;
    public trackWidth: number;

    private x: number = 0;
    private y: number = 0;
    private angle: number = 0;

    private vL: number = 0;
    private vR: number = 0;
    private velX: number = 0;
    private velY: number = 0;
    public maxAccel: number;
    public maxDecel: number;
    public lateralFriction: number;

    constructor(startX: number, startY: number, startAngle: number, width: number, height: number, maxSpeed: number, trackWidth: number, maxAccel: number, maxDecel: number, lateralFriction: number = 10) {
        this.x = startX;
        this.y = startY;
        this.angle = startAngle;
        this.width = width;
        this.height = height;
        this.maxSpeed = maxSpeed;
        this.trackWidth = trackWidth;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.lateralFriction = lateralFriction;
    }

    private setX(x: number) { 
        this.x = x 
    }

    private setY(y: number) { 
        this.y = y 
    }

    private setAngle(angle: number) { 
        this.angle = normalizeDeg(angle);
    }

    getX() { return this.x; }
    getY() { return this.y; }
    getAngle() { return this.angle; }

    // Returns Velocity in in/s (includes lateral drift)
    public getXVelocity(): number {
        return this.velX;
    }

    // Returns Velocity in in/s (includes lateral drift)
    public getYVelocity(): number {
        return this.velY;
    }

    private moveTowards(current: number, target: number, dt: number): number {
        const diff = target - current;

        const signFlip = current !== 0 && target !== 0 && Math.sign(current) !== Math.sign(target);
        const isAccel = !signFlip && Math.abs(target) > Math.abs(current);

        const maxDelta = (isAccel ? this.maxAccel : this.maxDecel) * dt;

        if (Math.abs(diff) <= maxDelta) return target;
        return current + Math.sign(diff) * maxDelta;
    }

    // Returns angular velocity in degrees per second
    public getAngularVelocity(): number {
        const vL_in = this.vL * 12;
        const vR_in = this.vR * 12;

        const b_in = this.trackWidth;

        if (b_in === 0) return 0;

        const omegaRad = (vR_in - vL_in) / b_in;

        return toDeg(omegaRad);
    }

    tankDrive(leftCmd: number, rightCmd: number, dt: number) {
        const b_in = this.trackWidth;      // Distance between wheel centers (inches)
        const v_max_ft = this.maxSpeed;    // Maximum linear velocity (feet/second)

        // Input is in the range -1 to 1, with - being reverse
        const left  = clamp(leftCmd,  -1, 1);
        const right = clamp(rightCmd, -1, 1);

        // Convert commands to wheel velocities (ft/s)
        const targetVL_ft = left  * v_max_ft;
        const targetVR_ft = right * v_max_ft;

        // Apply acceleration limits to smoothly approach target velocity
        this.vL = this.moveTowards(this.vL, targetVL_ft, dt);
        this.vR = this.moveTowards(this.vR, targetVR_ft, dt);

        // Convert wheel velocities to inches/second for position calculations
        const vL_in = this.vL * 12;
        const vR_in = this.vR * 12;

        // Calculate linear velocity (in/s)
        const v_in = (vR_in + vL_in) / 2;

        // Calculate angular velocity using differential drive kinematic equation
        const ω = (vL_in - vR_in) / b_in;

        // Update heading first
        const θ = toRad(this.getAngle());
        const θNew = θ + ω * dt;
        let θdegNew = toDeg(θNew);
        θdegNew = normalizeDeg(θdegNew);
        this.setAngle(θdegNew);

        // Forward and lateral unit vectors in the NEW heading direction
        const θUpdated = toRad(this.getAngle());
        const forwardX = Math.sin(θUpdated);
        const forwardY = Math.cos(θUpdated);
        const lateralX = Math.cos(θUpdated);    // perpendicular right
        const lateralY = -Math.sin(θUpdated);   // perpendicular right

        // Decompose current velocity into longitudinal and lateral components
        const longComponent = this.velX * forwardX + this.velY * forwardY;
        const latComponent  = this.velX * lateralX + this.velY * lateralY;

        // Longitudinal: set directly from wheel speeds (tires grip in drive direction)
        const newLong = v_in;

        // Lateral: decay with friction (simulates tire slip during turns)
        const newLat = latComponent * Math.max(0, 1 - this.lateralFriction * dt);

        // Reconstruct actual velocity vector
        this.velX = newLong * forwardX + newLat * lateralX;
        this.velY = newLong * forwardY + newLat * lateralY;

        // Update position using actual velocity
        this.setX(this.getX() + this.velX * dt);
        this.setY(this.getY() + this.velY * dt);
    }

    public stop() {
        this.vL = 0;
        this.vR = 0;
        this.velX = 0;
        this.velY = 0;
    }

    public setPose(x: number, y: number, angle: number) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.velX = 0;
        this.velY = 0;

        return true;
    }
}
