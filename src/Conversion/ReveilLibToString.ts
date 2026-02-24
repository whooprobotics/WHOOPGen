import type { ReveilLibConstants, revTurnConstants, revDriveConstants } from "../core/ReveiLibSim/RevConstants";
import { getUnequalRevConstants } from "../core/ReveiLibSim/RevConstants";
import { getDefaultConstants } from "../core/DefaultConstants";
import { toRevCoordinate } from "../core/ReveiLibSim/Util";
import type { Coordinate } from "../core/Types/Coordinate";
import { getBackwardsSnapPose, getForwardSnapPose, type Path } from "../core/Types/Path";
import { trimZeros } from "../core/Util";

const roundOff = (val: number | undefined | null | string, digits: number) => {
    if (val === null || val === undefined || typeof val === "string") return "";
    return trimZeros(val.toFixed(digits));
}

export function reveilLibToString(path: Path, selected: boolean = false) {
    let pathString: string = '';
    let startWrapper = false;

    for (let idx = 0; idx < path.segments.length; idx++) {
        const control = path.segments[idx];

        if (selected && !control.selected) continue;

        const kind = control.kind;
        const revCoords = toRevCoordinate(control.pose.x ?? 0, control.pose.y ?? 0);
        const x = roundOff(revCoords.x, 2);
        const y = roundOff(revCoords.y, 2);

        const angle = roundOff(control.pose.angle, 2);

        // const commandName = control.command.name; 
        // const commandPercent = roundOff(control.command.percent, 0);
        

        if (idx === 0) {
            pathString += (
                `  odom->set_position({${x}_in, ${y}_in, ${angle}_deg});`
            )        
            continue;
        }


        if (control.groupId !== undefined && !startWrapper) {
            pathString += (
                `\n  reckless->go({`
            )
            startWrapper = true;
        }

        if (control.groupId === undefined) {
            pathString += (
                `\n  reckless->go({`
            )
        }
        
        if (kind === "angleTurn" || kind === "angleSwing") {
            const k = control.constants.turn as ReveilLibConstants;
            pathString += (
                `
    &TurnSegment(
      ${roundOff(k.maxSpeed, 2)}, ${roundOff(k.stopCoastPower, 2)},
      ${angle}_deg,
      ${roundOff((k.stopHarshThreshold ?? 0) / 1000, 3)}, ${roundOff((k.stopCoastThreshold ?? 0) / 1000, 2)}, ${roundOff(k.brakeTime, 2)}_ms
    ),`
            )
        }

        if (kind === "pointTurn" || kind === "pointSwing") {
            const k = control.constants.turn as ReveilLibConstants;
            const previousPos = getBackwardsSnapPose(path, idx - 1);
            const turnToPos = getForwardSnapPose(path, idx);

            const pos: Coordinate =
            turnToPos
                ? { x: turnToPos.x ?? 0, y: turnToPos.y ?? 0 }
                : previousPos
                ? { x: previousPos.x ?? 0, y: (previousPos.y ?? 0) + 5 }
                : { x: 0, y: 5 };
            
            const turnX = roundOff(pos.x, 2);
            const turnY = roundOff(pos.y, 2);

            pathString += (
                `
    &LookAt(
      ${roundOff(k.maxSpeed, 2)}, ${roundOff(k.stopCoastPower, 2)},
      {${turnX}_in, ${turnY}_in}, ${roundOff(angle, 2)}_deg, ${roundOff(k.dropEarly, 2)}_deg,
      ${roundOff((k.stopHarshThreshold ?? 0) / 1000, 3)}, ${roundOff((k.stopCoastThreshold ?? 0) / 1000, 2)}, ${roundOff(k.brakeTime, 2)}_ms
    ),`
            )
        }

        if (kind === "pointDrive") {
            const k = control.constants.drive as ReveilLibConstants;
            pathString += (
                `
    &PilonsSegment(
      &ConstantMotion(${roundOff(k.maxSpeed, 2)}),
      &PilonsCorrection(${roundOff(k.kCorrection, 1)}, ${roundOff(k.maxError, 2)}_in),
      &SimpleStop(${roundOff(k.stopHarshThreshold, 0)}_ms, ${roundOff(k.stopCoastThreshold, 0)}_ms, ${roundOff(k.stopCoastPower, 2)}),
      {${x}_in, ${y}_in}, ${roundOff(k.dropEarly, 2)}_in
    ),`
            )          
        }

        if (kind === "poseDrive") {
            const k = control.constants.drive as ReveilLibConstants;
            pathString += (
                `
    &BoomerangSegment(
      &ConstantMotion(${roundOff(k.maxSpeed, 2)}),
      &PilonsCorrection(${roundOff(k.kCorrection, 1)}, ${roundOff(k.maxError, 2)}_in),
      &SimpleStop(${roundOff(k.stopHarshThreshold, 0)}_ms, ${roundOff(k.stopCoastThreshold, 0)}_ms, ${roundOff(k.stopCoastPower, 2)}),
      ${roundOff(k.lead, 2)},
      {${x}_in, ${y}_in, ${angle}_deg}, ${roundOff(k.dropEarly, 2)}_in
    ),`
            )     
        }

        if (startWrapper && path.segments[idx + 1]?.groupId !== control.groupId) {
            pathString += (
                `\n  });`
            )            
            startWrapper = false;
        }

        if (control.groupId === undefined) {
            pathString += (
                `\n  });`
            )        
        }

    }

    if (selected) pathString = pathString.startsWith("\n") ? pathString.slice(1) : pathString;
    return pathString;
}

export function revToString(path: Path, selected: boolean = false) {
    let pathString: string = '';

    for (let idx = 0; idx < path.segments.length; idx++) {
        const control = path.segments[idx];

        if (selected && !control.selected) continue;

        const kind = control.kind;
        const revCoords = toRevCoordinate(control.pose.x ?? 0, control.pose.y ?? 0);
        const x = roundOff(revCoords.x, 2);
        const y = roundOff(revCoords.y, 2);
        const angle = roundOff(control.pose.angle, 2);

        if (idx === 0) {
            pathString += `  set_pose(${x}, ${y}, ${angle});`;
            continue;
        }

        if (kind === "angleTurn") {
            const k = control.constants.turn as ReveilLibConstants;
            const defaults = (getDefaultConstants("ReveilLib", "angleTurn") as revTurnConstants).turn;
            const diff = getUnequalRevConstants(defaults, k);
            const params: string[] = [];

            if (diff.maxSpeed != null)
                params.push(`.speed = ${roundOff(diff.maxSpeed, 2)}`);
            if (diff.stopCoastPower != null)
                params.push(`.min = ${roundOff(diff.stopCoastPower, 2)}`);
            if (diff.stopCoastThreshold != null)
                params.push(`.coast = ${roundOff(diff.stopCoastThreshold, 0)}`);
            if (diff.stopHarshThreshold != null)
                params.push(`.harsh = ${roundOff(diff.stopHarshThreshold, 0)}`);
            if (diff.brakeTime != null)
                params.push(`.time = ${roundOff(diff.brakeTime, 0)}`);

            pathString += params.length === 0
                ? `\n  turn(${angle});`
                : params.length === 1
                ? `\n  turn(${angle}, { ${params[0]} });`
                : `\n  turn(${angle}, {\n${params.map(p => `    ${p}`).join(",\n")}\n  });`;
        }

        if (kind === "pointTurn") {
            const k = control.constants.turn as ReveilLibConstants;
            const defaults = (getDefaultConstants("ReveilLib", "pointTurn") as revTurnConstants).turn;
            const diff = getUnequalRevConstants(defaults, k);
            const params: string[] = [];

            const previousPos = getBackwardsSnapPose(path, idx - 1);
            const turnToPos = getForwardSnapPose(path, idx);

            const pos: Coordinate =
            turnToPos
                ? { x: turnToPos.x ?? 0, y: turnToPos.y ?? 0 }
                : previousPos
                ? { x: previousPos.x ?? 0, y: (previousPos.y ?? 0) + 5 }
                : { x: 0, y: 5 };

            const revCoords = toRevCoordinate(turnToPos?.x ?? 0, turnToPos?.y ?? 0);

            const turnX = roundOff(revCoords.x, 2);
            const turnY = roundOff(revCoords.y, 2);

            if (angle !== null && Number(angle) !== 0)
                params.push(`.offset = ${angle}`)
            if (diff.maxSpeed != null)
                params.push(`.speed = ${roundOff(diff.maxSpeed, 2)}`);
            if (diff.stopCoastPower != null)
                params.push(`.min = ${roundOff(diff.stopCoastPower, 2)}`);
            if (diff.stopCoastThreshold != null)
                params.push(`.coast = ${roundOff(diff.stopCoastThreshold, 0)}`);
            if (diff.stopHarshThreshold != null)
                params.push(`.harsh = ${roundOff(diff.stopHarshThreshold, 0)}`);
            if (diff.brakeTime != null)
                params.push(`.time = ${roundOff(diff.brakeTime, 0)}`);

            pathString += params.length === 0
                ? `\n  look(${turnX}, ${turnY});`
                : params.length === 1
                ? `\n  look(${turnX}, ${turnY}, { ${params[0]} });`
                : `\n  look(${turnX}, ${turnY}, {\n${params.map(p => `    ${p}`).join(",\n")}\n  });`;
        }

        if (kind === "pointDrive") {
            const k = control.constants.drive as ReveilLibConstants;
            const defaults = (getDefaultConstants("ReveilLib", "pointDrive") as revDriveConstants).drive;
            const diff = getUnequalRevConstants(defaults, k);
            const params: string[] = [];

            if (diff.maxSpeed != null)
                params.push(`.speed = ${roundOff(diff.maxSpeed, 2)}`);
            if (diff.stopCoastPower != null)
                params.push(`.min = ${roundOff(diff.stopCoastPower, 2)}`);
            if (diff.kCorrection != null)
                params.push(`.correction = ${roundOff(diff.kCorrection, 1)}`);
            if (diff.maxError != null)
                params.push(`.error = ${roundOff(diff.maxError, 2)}`);
            if (diff.stopCoastThreshold != null)
                params.push(`.coast = ${roundOff(diff.stopCoastThreshold, 0)}`);
            if (diff.stopHarshThreshold != null)
                params.push(`.harsh = ${roundOff(diff.stopHarshThreshold, 0)}`);

            pathString += params.length === 0
                ? `\n  move(${x}, ${y});`
                : params.length === 1
                ? `\n  move(${x}, ${y}, { ${params[0]} });`
                : `\n  move(${x}, ${y}, {\n${params.map(p => `    ${p}`).join(",\n")}\n  });`;
        }
    }

    if (selected) pathString = pathString.startsWith("\n") ? pathString.slice(1) : pathString;
    return pathString;
}