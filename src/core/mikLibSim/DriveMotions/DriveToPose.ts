import type { Robot } from "../../Robot";
import { clamp, toDeg, toRad } from "../../Util";
import { kMikLibSpeed } from "../MikConstants";
import type { PID } from "../PID";
import { clamp_max_slip, clamp_min_voltage, is_line_settled, left_voltage_scaling, overturn_scaling, reduce_negative_180_to_180, reduce_negative_90_to_90, right_voltage_scaling, slew_scaling } from "../Util";

let posePrevLineSettled: boolean | null = null;
let poseInitCenterLineSide: boolean | null = null;
let posePrevDriveOutput: number | null = null;
let poseCrossedCenterLine = false;
let poseClose: boolean = false;

function resetDriveToPose(drivePID: PID, headingPID: PID) {
    drivePID.reset();
    headingPID.reset();
    posePrevLineSettled = null;
    poseInitCenterLineSide = null;
    posePrevDriveOutput = null;
    poseCrossedCenterLine = false;
    poseClose = false;
}

export function driveToPose(robot: Robot, dt: number, x: number, y: number, angle: number, drivePID: PID, headingPID: PID): boolean {
  if (posePrevLineSettled === null || poseInitCenterLineSide === null) {
    posePrevLineSettled = is_line_settled(x, y, angle, robot.getX(), robot.getY());
    poseInitCenterLineSide = is_line_settled(x, y, angle + 90, robot.getX(), robot.getY());
    poseCrossedCenterLine = false;
  }

  if (drivePID.isSettled()) {
    resetDriveToPose(drivePID, headingPID);
    return true;
  }

  const pose_line_settled = is_line_settled(x, y, angle, robot.getX(), robot.getY());
  if (pose_line_settled && !posePrevLineSettled) {
    resetDriveToPose(drivePID, headingPID);
    return true;
  }
  posePrevLineSettled = pose_line_settled;

  const centerSide = is_line_settled(x, y, angle + 90, robot.getX(), robot.getY());
  poseCrossedCenterLine ||= (centerSide !== poseInitCenterLineSide);

  const target_distance = Math.hypot(x - robot.getX(), y - robot.getY());

  const carrot_X = x - Math.sin(toRad(angle)) * (drivePID.lead * target_distance + drivePID.setback);
  const carrot_Y = y - Math.cos(toRad(angle)) * (drivePID.lead * target_distance + drivePID.setback);

  let drive_error = Math.hypot(carrot_X - robot.getX(), carrot_Y - robot.getY());
  let heading_error = reduce_negative_180_to_180(
    toDeg(Math.atan2(carrot_X - robot.getX(), carrot_Y - robot.getY())) - robot.getAngle()
  );
  let driveSign = 1;

  
  if (drive_error < drivePID.settleError || poseCrossedCenterLine || drive_error < drivePID.setback) {
    heading_error = reduce_negative_180_to_180(angle - robot.getAngle());
    drive_error = target_distance;
    poseClose = true;
  }
  
  if (drivePID.driveDirection === "reverse") {
    heading_error = reduce_negative_180_to_180(heading_error + 180);
    driveSign = -1;
  }

  let drive_output = drivePID.compute(drive_error);

  let heading_scale_factor = Math.cos(toRad(heading_error));
  if (drivePID.driveDirection !== null) heading_scale_factor = Math.max(0, heading_scale_factor);

  drive_output *= heading_scale_factor * driveSign;

  if (drivePID.driveDirection === null) heading_error = reduce_negative_90_to_90(heading_error);
  let heading_output = headingPID.compute(heading_error);

  drive_output = clamp(
    drive_output,
    -Math.abs(heading_scale_factor) * drivePID.maxSpeed,
    Math.abs(heading_scale_factor) * drivePID.maxSpeed
  );
  heading_output = clamp(heading_output, -headingPID.maxSpeed, headingPID.maxSpeed);

  console.log(poseClose);
  drive_output = slew_scaling(drive_output, posePrevDriveOutput ?? 0, drivePID.slew * (dt / 0.01), !poseClose); // normalizes dt from 16ms to 10ms to match mikLib

  drive_output = clamp_max_slip(drive_output, robot.getX(), robot.getY(), robot.getAngle(), carrot_X, carrot_Y, drivePID.drift);

  drive_output = overturn_scaling(drive_output, heading_output, drivePID.maxSpeed);

  drive_output = clamp_min_voltage(drive_output, drivePID.minSpeed);

  robot.tankDrive(
    left_voltage_scaling(drive_output, heading_output) / kMikLibSpeed,
    right_voltage_scaling(drive_output, heading_output) / kMikLibSpeed,
    dt
  );

  posePrevDriveOutput = drive_output;

  return false;
}
