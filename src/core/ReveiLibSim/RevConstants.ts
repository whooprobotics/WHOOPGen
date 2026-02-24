 export interface ReveilLibConstants {
    maxSpeed: number | null;

    kCorrection: number | null;
    maxError: number | null;
    
    stopHarshThreshold: number | null;
    stopCoastThreshold: number | null;
    stopCoastPower: number | null;
    stopTimeout: number | null;
    brakeTime: number | null;

    dropEarly: number | null;

    lead: number | null;
}

export type revDriveConstants = {
    drive: ReveilLibConstants;
};

export type revTurnConstants = {
    turn: ReveilLibConstants;
};

export const cloneKRev = (c: ReveilLibConstants): ReveilLibConstants => ({ ...c });

export function getUnequalRevConstants(defaults: ReveilLibConstants, actual: ReveilLibConstants): Partial<ReveilLibConstants> {
    const out: Partial<ReveilLibConstants> = {};
    if (defaults.maxSpeed !== actual.maxSpeed) out.maxSpeed = actual.maxSpeed;
    if (defaults.kCorrection !== actual.kCorrection) out.kCorrection = actual.kCorrection;
    if (defaults.maxError !== actual.maxError) out.maxError = actual.maxError;
    if (defaults.stopHarshThreshold !== actual.stopHarshThreshold) out.stopHarshThreshold = actual.stopHarshThreshold;
    if (defaults.stopCoastThreshold !== actual.stopCoastThreshold) out.stopCoastThreshold = actual.stopCoastThreshold;
    if (defaults.stopCoastPower !== actual.stopCoastPower) out.stopCoastPower = actual.stopCoastPower;
    if (defaults.stopTimeout !== actual.stopTimeout) out.stopTimeout = actual.stopTimeout;
    if (defaults.brakeTime !== actual.brakeTime) out.brakeTime = actual.brakeTime;
    if (defaults.dropEarly !== actual.dropEarly) out.dropEarly = actual.dropEarly;
    if (defaults.lead !== actual.lead) out.lead = actual.lead;
    return out;
}

function createRevConstants(values: Partial<ReveilLibConstants> = {}): ReveilLibConstants {
    return {
        maxSpeed: values.maxSpeed ?? null,

        kCorrection: values.kCorrection ?? null,
        maxError: values.maxError ?? null,

        stopHarshThreshold: values.stopHarshThreshold ?? null,
        stopCoastThreshold: values.stopCoastThreshold ?? null,
        stopCoastPower: values.stopCoastPower ?? null,
        stopTimeout: values.stopTimeout ?? null,
        brakeTime: values.brakeTime ?? null,

        dropEarly: values.dropEarly ?? null,
        lead: values.lead ?? null
    };
}

export const kPilon: ReveilLibConstants = createRevConstants({ 
    maxSpeed: 1,
    kCorrection: 2,
    maxError: 0.5,
    stopHarshThreshold: 60,
    stopCoastThreshold: 200,
    stopCoastPower: 0.25,
    brakeTime: 250,
    dropEarly: 0, 
});

export const kTurn: ReveilLibConstants = createRevConstants({ 
    maxSpeed: 0.75,
    stopHarshThreshold: 60,
    stopCoastThreshold: 200,
    stopCoastPower: 0.25,
    brakeTime: 100,
    dropEarly: 0,
});

export const kLootAt: ReveilLibConstants = createRevConstants({ 
    maxSpeed: 0.75,
    stopHarshThreshold: 60,
    stopCoastThreshold: 200,
    stopCoastPower: 0.25,
    brakeTime: 100,
    dropEarly: 0,
})

export const kBoomerang: ReveilLibConstants = createRevConstants({
    maxSpeed: 0.75,
    kCorrection: 2,
    maxError: 0.5,
    stopHarshThreshold: 60,
    stopCoastThreshold: 200,
    stopCoastPower: 0.25,
    brakeTime: 250,
    dropEarly: 0, 
    lead: 0.4,
});