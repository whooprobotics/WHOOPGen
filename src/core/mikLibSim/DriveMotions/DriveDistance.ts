import { clamp, toRad } from "../../Util";
import type { Robot } from "../../Robot";
import type { PID } from "../PID";
import { clamp_min_voltage, reduce_negative_180_to_180, slew_scaling } from "../Util";

let driveDistanceStartX: number | null = null;
let driveDistanceStartY: number | null = null;
let prevDriveOutput: number | null = null;
let prevHeadingOutput: number | null = null;

export function resetDriveDistance(drivePID: PID, headingPID: PID) {
    driveDistanceStartX = null;
    driveDistanceStartY = null;
    prevDriveOutput = null;
    prevHeadingOutput = null;
    drivePID.reset();
    headingPID.reset();
}

export function driveDistance(robot: Robot, dt: number, distance: number, heading: number, drivePID: PID, headingPID: PID) {
    if (driveDistanceStartX === null || driveDistanceStartY === null) {
        driveDistanceStartX = robot.getX();
        driveDistanceStartY = robot.getY();
    }

    const dx = robot.getX() - driveDistanceStartX;
    const dy = robot.getY() - driveDistanceStartY;

    const traveled = dx * Math.sin(toRad(heading)) + dy * Math.cos(toRad(heading));

    const drive_error = distance - traveled;

    const heading_error = reduce_negative_180_to_180(heading - robot.getAngle());

    let drive_output = drivePID.compute(drive_error);
    let heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drivePID.maxSpeed, drivePID.maxSpeed);
    heading_output = clamp(heading_output, -headingPID.maxSpeed, headingPID.maxSpeed);

    drive_output = slew_scaling(drive_output, prevDriveOutput ?? 0, drivePID.slew * (dt / 0.01), Math.abs(drive_error) > drivePID.settleError);
    heading_output = slew_scaling(heading_output, prevHeadingOutput ?? 0, headingPID.slew * (dt / 0.01));

    drive_output = clamp_min_voltage(drive_output, drivePID.minSpeed);

    if (drivePID.isSettled()) {
        resetDriveDistance(drivePID, headingPID);
        return true;
    }

    robot.tankDrive((drive_output + heading_output) / 12, (drive_output - heading_output) / 12, dt);

    prevDriveOutput = drive_output;
    prevHeadingOutput = heading_output;

    return false;
}
