import type { Robot } from "../../Robot";
import { clamp } from "../../Util";
import { kMikLibSpeed } from "../MikConstants";
import type { PID } from "../PID";
import { angle_error, clamp_min_voltage, slew_scaling } from "../Util";

let crossed: boolean = false;
let prevError: number | null = null; 
let prevRawError: number | null = null;
let prevOutput: number | null = null;

function resetSwingToAngle(swingPID: PID) {
    crossed = false;
    prevOutput = null;
    prevError = null;
    prevRawError = null;
    swingPID.reset();
}

export function swingToAngle(robot: Robot, dt: number, angle: number, swingPID: PID) {
    const rawError = angle_error(angle - robot.getAngle(), null);
    let error = angle_error(angle - robot.getAngle(), swingPID.turnDirection);
    
    if (prevError === null || prevRawError === null) {
        prevError = error;
        prevRawError = rawError;
    }
    
    if (Math.sign(rawError) != Math.sign(prevRawError)) {
        crossed = true;
    }
    prevRawError = rawError;
    
    if (crossed) {
        error = rawError;
    } else {
        error = angle_error(angle - robot.getAngle(), swingPID.turnDirection);
    }
    
    if (swingPID.minSpeed != 0 && crossed && Math.sign(error) != Math.sign(prevError)) {
        resetSwingToAngle(swingPID);
        return true;
    }
    prevError = error;

    let output = swingPID.compute(error);
    
    if (swingPID.isSettled()) {
        resetSwingToAngle(swingPID);
        return true;
    }

    output = clamp(output, -swingPID.maxSpeed, swingPID.maxSpeed);
    output = slew_scaling(output, prevOutput ?? 0, swingPID.slew * (dt / 0.01), Math.abs(error) > swingPID.starti);
    output = clamp_min_voltage(output, swingPID.minSpeed);
    prevOutput = output;

    if (swingPID.swingDirection === "left") {
        robot.tankDrive(output / kMikLibSpeed, 0, dt);
    } else {
        robot.tankDrive(0, -output / kMikLibSpeed, dt);
    }
    
    return false;
}

