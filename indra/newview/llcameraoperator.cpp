/**
 * @file llcameraoperator.cpp
 * @brief Procedural handheld camera operator for the flycam (VCHH port).
 *
 * $LicenseInfo:firstyear=2026&license=viewerlgpl$
 * Phoenix / Firestorm Viewer Source Code
 * $/LicenseInfo$
 */

#include "llviewerprecompiledheaders.h"

#include "llcameraoperator.h"

#include <cmath>

#include "llviewercontrol.h"    // gSavedSettings, LLCachedControl
#include "llmath.h"             // DEG_TO_RAD, F_PI, F_TWO_PI, llclamp

// ---------------------------------------------------------------------------
// Constants (ported from VCHH v3.4)
// ---------------------------------------------------------------------------
namespace
{
constexpr F32 DT_REF        = 1.f / 30.f;   // authored per-frame retains map to 30 fps
constexpr F32 ONSET_DECAY   = 0.55f;        // per-frame retain @30fps
constexpr F32 ONSET_KICK    = 5.f;
constexpr F32 SETTLE_KICK   = 5.f;
constexpr F32 REACT_MAX     = 4.f;
constexpr F32 PHASE_WRAP    = 256.f;        // hash lattice period; wrap is seamless
// The shader authored reactive magnitudes in viewport fractions; one full
// viewport is roughly one radian of view angle, so fraction units convert
// to camera radians with a factor of ~1.
constexpr F32 FRAC_TO_RAD   = 1.0f;

inline F32 vc_frac(F32 x)                   { return x - floorf(x); }
inline F32 vc_lerp(F32 a, F32 b, F32 u)     { return a + (b - a) * u; }
inline F32 vc_sat(F32 x)                    { return llclamp(x, 0.f, 1.f); }
inline F32 vc_smoothstep(F32 e0, F32 e1, F32 x)
{
    F32 t = vc_sat((x - e0) / llmax(e1 - e0, 1e-6f));
    return t * t * (3.f - 2.f * t);
}
// quintic ease (C2), no overshoot
inline F32 vc_quintic(F32 u)                { return u * u * u * (u * (u * 6.f - 15.f) + 10.f); }

// frametime-normalized EMA retain: per-frame retain authored @30fps -> dt
inline F32 vc_retain(F32 perFrameRetain30, F32 dtS)
{
    F32 r   = llclamp(perFrameRetain30, 0.02f, 0.9995f);
    F32 tau = -DT_REF / logf(r);
    return expf(-dtS / llmax(tau, 1e-4f));
}

inline F32 vc_wrap(F32 x)                   { return x - floorf(x / PHASE_WRAP) * PHASE_WRAP; }

// ---------------------------------------------------------------------------
// value noise / fBm -- periodic lattice hash, quintic (C2) interpolation
// ---------------------------------------------------------------------------
F32 vc_hash(F32 p)
{
    p = p - floorf(p / PHASE_WRAP) * PHASE_WRAP;    // lattice period 256
    p = vc_frac(p * 0.1031f);
    p *= p + 33.33f;
    p *= p + p;
    return vc_frac(p);
}

F32 vc_vnoise5(F32 x)
{
    F32 i = floorf(x);
    F32 f = vc_frac(x);
    F32 u = vc_quintic(f);
    return vc_lerp(vc_hash(i), vc_hash(i + 1.f), u);
}

// integer octave ratios keep the wrapped phase continuous across the wrap
F32 vc_fbm(F32 x, F32 hiGain)
{
    F32 s =                  (vc_vnoise5(x)               - 0.5f);
    s    += 0.50f * hiGain * (vc_vnoise5(x * 2.f + 11.7f) - 0.5f);
    s    += 0.25f * hiGain * (vc_vnoise5(x * 4.f + 23.3f) - 0.5f);
    return s * 1.4f;
}

F32 vc_fbmSmooth(F32 x, F32 hiGain)
{
    F32 s =                  (vc_vnoise5(x)               - 0.5f);
    s    += 0.45f * hiGain * (vc_vnoise5(x * 2.f + 11.7f) - 0.5f);
    s    += 0.18f * hiGain * (vc_vnoise5(x * 3.f + 23.3f) - 0.5f);
    return s * 1.5f;
}

// sub-Hz sine weave; ratios are exact /256 rationals so seamless on wrap
F32 vc_weave(F32 tf, F32 seed)
{
    F32 s = 0.62f * sinf(tf * (0.625f    * F_TWO_PI) + seed);
    s    += 0.31f * sinf(tf * (1.0f      * F_TWO_PI) + seed * 1.7f + 1.3f);
    s    += 0.17f * sinf(tf * (0.390625f * F_TWO_PI) + seed * 0.6f + 4.1f);
    return s;
}

// sparse operator re-framing (per-window eased retarget)
void vc_recompose(F32 rp, F32 sd, F32 interval, F32& outX, F32& outY)
{
    F32 w  = floorf(rp);
    F32 k0 = w + sd * 0.37f;
    F32 px = vc_hash(k0 - 1.f)         - 0.5f;
    F32 py = vc_hash(k0 - 1.f + 91.7f) - 0.5f;
    F32 cx = vc_hash(k0)               - 0.5f;
    F32 cy = vc_hash(k0 + 91.7f)       - 0.5f;
    F32 fE = 0.15f + 0.55f * vc_hash(k0 + 47.3f);
    F32 blendW = 0.45f / llmax(interval, 0.5f);
    F32 u = vc_quintic(vc_sat((vc_frac(rp) - fE) / llmax(blendW, 0.02f)));
    outX = vc_lerp(px, cx, u);
    outY = vc_lerp(py, cy, u);
}

// ---------------------------------------------------------------------------
// Personality (Operator Style) -- ported verbatim from VCHH
// ---------------------------------------------------------------------------
struct Persona
{
    F32 ampMul, freqMul, hiMul, transMul, rollMul, breathMul;
    F32 energyGain, onsetGain, settleGain, drag, smoothing;
    F32 walkCouple, walkRate, motionCalm;
};

Persona getPersona(S32 s)
{
    Persona p;
    p.ampMul=1.f; p.freqMul=1.f; p.hiMul=1.f; p.transMul=1.f; p.rollMul=1.f; p.breathMul=1.f;
    p.energyGain=2.5f; p.onsetGain=1.6f; p.settleGain=2.f; p.drag=0.010f; p.smoothing=0.85f;
    p.walkCouple=0.f; p.walkRate=1.8f; p.motionCalm=0.4f;
    switch (s)
    {
        case 1: // Tripod / Sticks
            p.ampMul=0.15f; p.freqMul=0.6f; p.hiMul=0.2f; p.transMul=0.1f; p.rollMul=0.1f; p.breathMul=0.6f;
            p.energyGain=0.5f; p.onsetGain=0.5f; p.settleGain=1.f; p.drag=0.f; p.smoothing=0.92f;
            p.walkCouple=0.f; p.walkRate=1.8f; p.motionCalm=0.5f; break;
        case 2: // Subtle Handheld
            p.ampMul=0.5f; p.freqMul=0.8f; p.hiMul=0.4f; p.transMul=0.6f; p.rollMul=0.4f; p.breathMul=1.f;
            p.energyGain=1.5f; p.onsetGain=1.f; p.settleGain=1.6f; p.drag=0.006f; p.smoothing=0.88f;
            p.walkCouple=0.f; p.walkRate=1.7f; p.motionCalm=0.5f; break;
        case 3: // Documentary
            p.ampMul=1.f; p.freqMul=1.f; p.hiMul=1.f; p.transMul=1.f; p.rollMul=0.8f; p.breathMul=1.f;
            p.energyGain=2.2f; p.onsetGain=1.8f; p.settleGain=2.2f; p.drag=0.012f; p.smoothing=0.84f;
            p.walkCouple=2.5f; p.walkRate=1.8f; p.motionCalm=0.5f; break;
        case 4: // Run & Gun
            p.ampMul=1.7f; p.freqMul=1.5f; p.hiMul=1.8f; p.transMul=1.2f; p.rollMul=1.3f; p.breathMul=1.1f;
            p.energyGain=4.f; p.onsetGain=3.2f; p.settleGain=2.6f; p.drag=0.018f; p.smoothing=0.78f;
            p.walkCouple=4.f; p.walkRate=2.3f; p.motionCalm=0.2f; break;
        case 5: // Shoulder Rig
            p.ampMul=0.9f; p.freqMul=0.7f; p.hiMul=0.35f; p.transMul=1.f; p.rollMul=0.6f; p.breathMul=0.9f;
            p.energyGain=2.f; p.onsetGain=1.4f; p.settleGain=2.6f; p.drag=0.022f; p.smoothing=0.90f;
            p.walkCouple=1.8f; p.walkRate=1.5f; p.motionCalm=0.6f; break;
        case 6: // Gimbal Float
            p.ampMul=0.6f; p.freqMul=0.4f; p.hiMul=0.1f; p.transMul=0.3f; p.rollMul=0.2f; p.breathMul=0.8f;
            p.energyGain=0.8f; p.onsetGain=0.5f; p.settleGain=1.f; p.drag=0.004f; p.smoothing=0.94f;
            p.walkCouple=0.f; p.walkRate=1.8f; p.motionCalm=0.85f; break;
        case 7: // Verite (Chaotic)
            p.ampMul=1.3f; p.freqMul=1.3f; p.hiMul=1.5f; p.transMul=1.1f; p.rollMul=1.1f; p.breathMul=1.f;
            p.energyGain=3.f; p.onsetGain=2.8f; p.settleGain=1.8f; p.drag=0.014f; p.smoothing=0.80f;
            p.walkCouple=3.2f; p.walkRate=2.f; p.motionCalm=0.25f; break;
        case 8: // Breathing Only
            p.ampMul=1.f; p.freqMul=0.7f; p.hiMul=0.3f; p.transMul=0.f; p.rollMul=0.15f; p.breathMul=1.3f;
            p.energyGain=0.3f; p.onsetGain=0.3f; p.settleGain=0.6f; p.drag=0.f; p.smoothing=0.90f;
            p.walkCouple=0.f; p.walkRate=1.8f; p.motionCalm=0.5f; break;
        default: break; // 0 = Custom (neutral defaults above)
    }
    return p;
}

// ---------------------------------------------------------------------------
// Motion Profile -- ported verbatim from VCHH
// ---------------------------------------------------------------------------
struct Profile { F32 energyMul, onsetMul, settleMul, dragMul, walkMul, calmMul, smoothMul; };

Profile getProfile(S32 s)
{
    Profile q;
    q.energyMul=1.f; q.onsetMul=1.f; q.settleMul=1.f; q.dragMul=1.f; q.walkMul=1.f; q.calmMul=1.f; q.smoothMul=1.f;
    switch (s)
    {
        case 1: q.energyMul=0.f; q.onsetMul=0.f; q.settleMul=0.f; q.dragMul=0.f; q.walkMul=0.f; break; // Locked
        case 2: q.energyMul=0.8f; q.onsetMul=0.6f; q.settleMul=1.2f; q.dragMul=2.2f; q.walkMul=0.6f; q.calmMul=1.1f; q.smoothMul=1.05f; break; // Follow
        case 3: q.energyMul=1.5f; q.onsetMul=1.6f; q.settleMul=0.8f; q.dragMul=1.f;  q.walkMul=1.3f; q.calmMul=0.6f; q.smoothMul=0.9f;  break; // Restless
        case 4: q.energyMul=0.7f; q.onsetMul=0.5f; q.settleMul=1.9f; q.dragMul=0.9f; q.walkMul=0.8f; q.calmMul=1.4f; q.smoothMul=1.05f; break; // Composed
        case 5: q.energyMul=0.5f; q.onsetMul=0.3f; q.settleMul=0.9f; q.dragMul=0.5f; q.walkMul=0.4f; q.calmMul=1.3f; q.smoothMul=1.1f;  break; // Drift
        case 6: q.energyMul=1.2f; q.onsetMul=2.4f; q.settleMul=0.7f; q.dragMul=1.1f; q.walkMul=1.f;  q.calmMul=0.9f; q.smoothMul=0.85f; break; // Snap
        case 7: q.energyMul=1.f;  q.onsetMul=1.f;  q.settleMul=1.f;  q.dragMul=0.8f; q.walkMul=1.9f; q.calmMul=1.f;  q.smoothMul=1.f;   break; // March
        default: break; // 0 = Natural
    }
    return q;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
LLCameraOperator& LLCameraOperator::instance()
{
    static LLCameraOperator sInstance;
    return sInstance;
}

void LLCameraOperator::reset()
{
    mSpeed = 0.f;
    mVecX = mVecY = 0.f;
    mOnsetEnv = mSettleEnv = 0.f;
    mLatchX = mLatchY = 0.f;
    // phases are left running -- restarting them would pop the idle motion;
    // they are seamless by construction (wrapped, periodic lattice)
}

LLCameraOperatorOutput LLCameraOperator::update(const LLCameraOperatorInput& input)
{
    LLCameraOperatorOutput out;

    // ---- settings (live) --------------------------------------------------
    static LLCachedControl<S32> style(gSavedSettings, "FlycamOperatorStyle", 3);
    static LLCachedControl<S32> profile(gSavedSettings, "FlycamOperatorProfile", 0);
    static LLCachedControl<F32> profileInfluence(gSavedSettings, "FlycamOperatorProfileInfluence", 0.7f);
    static LLCachedControl<F32> master(gSavedSettings, "FlycamOperatorMaster", 1.f);
    static LLCachedControl<F32> reactivity(gSavedSettings, "FlycamOperatorReactivity", 1.f);
    static LLCachedControl<F32> idleIntensity(gSavedSettings, "FlycamOperatorIdleIntensity", 1.f);
    static LLCachedControl<F32> tremorDamping(gSavedSettings, "FlycamOperatorTremorDamping", 0.6f);
    static LLCachedControl<F32> transFreq(gSavedSettings, "FlycamOperatorPanTiltFreq", 0.6f);
    static LLCachedControl<F32> panAmount(gSavedSettings, "FlycamOperatorPanAmount", 0.6f);        // deg
    static LLCachedControl<S32> lateralMode(gSavedSettings, "FlycamOperatorPanMode", 0);
    static LLCachedControl<F32> lateralSmoothness(gSavedSettings, "FlycamOperatorPanSmoothness", 0.6f);
    static LLCachedControl<F32> lateralPanBoost(gSavedSettings, "FlycamOperatorPanReactSmoothing", 0.5f);
    static LLCachedControl<F32> tiltAmount(gSavedSettings, "FlycamOperatorTiltAmount", 0.6f);      // deg
    static LLCachedControl<F32> rollFreq(gSavedSettings, "FlycamOperatorRollFreq", 0.3f);
    static LLCachedControl<F32> rollAmount(gSavedSettings, "FlycamOperatorRollAmount", 0.5f);      // deg
    static LLCachedControl<F32> rollSmoothness(gSavedSettings, "FlycamOperatorRollDamping", 0.65f);
    static LLCachedControl<F32> breathFreq(gSavedSettings, "FlycamOperatorBreathFreq", 0.25f);
    static LLCachedControl<F32> breathAmount(gSavedSettings, "FlycamOperatorBreathAmount", 1.5f);  // %
    static LLCachedControl<F32> breathLift(gSavedSettings, "FlycamOperatorBreathLift", 0.2f);
    static LLCachedControl<F32> recomposeAmount(gSavedSettings, "FlycamOperatorRecomposeAmount", 0.35f); // deg
    static LLCachedControl<F32> recomposeInterval(gSavedSettings, "FlycamOperatorRecomposeInterval", 7.f);
    static LLCachedControl<F32> timeSpeed(gSavedSettings, "FlycamOperatorTimeSpeed", 1.f);
    static LLCachedControl<F32> seed(gSavedSettings, "FlycamOperatorSeed", 0.f);
    // reactive dynamics
    static LLCachedControl<F32> motionPan(gSavedSettings, "FlycamOperatorMotionPan", 1.f);
    static LLCachedControl<F32> motionTilt(gSavedSettings, "FlycamOperatorMotionTilt", 1.f);
    static LLCachedControl<F32> motionRoll(gSavedSettings, "FlycamOperatorMotionRoll", 1.f);
    static LLCachedControl<F32> motionBreath(gSavedSettings, "FlycamOperatorMotionBreath", 1.f);
    static LLCachedControl<F32> onsetAmount(gSavedSettings, "FlycamOperatorOnset", 1.f);
    static LLCachedControl<F32> dragAmount(gSavedSettings, "FlycamOperatorDrag", 1.f);
    static LLCachedControl<F32> settleAmount(gSavedSettings, "FlycamOperatorSettle", 1.f);
    static LLCachedControl<F32> settleFreq(gSavedSettings, "FlycamOperatorSettleFreq", 8.f);
    static LLCachedControl<F32> settleDecay(gSavedSettings, "FlycamOperatorSettleDecay", 0.85f);
    // walk & gait
    static LLCachedControl<bool> forceWalk(gSavedSettings, "FlycamOperatorForceWalk", false);
    static LLCachedControl<F32> walkCadence(gSavedSettings, "FlycamOperatorWalkCadence", 1.f);
    static LLCachedControl<F32> cadenceDrive(gSavedSettings, "FlycamOperatorCadenceDrive", 0.25f);
    static LLCachedControl<F32> forwardWalkBias(gSavedSettings, "FlycamOperatorForwardWalkBias", 1.f);
    static LLCachedControl<F32> lateralWalkBias(gSavedSettings, "FlycamOperatorLateralWalkBias", 0.25f);
    static LLCachedControl<F32> stepBob(gSavedSettings, "FlycamOperatorStepBob", 0.015f);          // m
    static LLCachedControl<F32> lateralStep(gSavedSettings, "FlycamOperatorLateralStep", 0.008f);  // m
    static LLCachedControl<F32> stepRoll(gSavedSettings, "FlycamOperatorStepRoll", 0.35f);         // deg
    // advanced
    static LLCachedControl<F32> energyTrim(gSavedSettings, "FlycamOperatorEnergyTrim", 1.f);
    static LLCachedControl<F32> motionCalmTrim(gSavedSettings, "FlycamOperatorMotionCalm", 1.f);
    static LLCachedControl<F32> smoothingTrim(gSavedSettings, "FlycamOperatorSmoothing", 1.f);
    static LLCachedControl<F32> gaitCoupling(gSavedSettings, "FlycamOperatorGaitCoupling", 1.f);
    // normalization references for the ground-truth speed metric
    static LLCachedControl<F32> refLinear(gSavedSettings, "FlycamOperatorRefLinearSpeed", 3.f);    // m/s
    static LLCachedControl<F32> refAngular(gSavedSettings, "FlycamOperatorRefAngularSpeed", 60.f); // deg/s

    const Persona P = getPersona(llclamp((S32)style, 0, 8));
    const Profile Q = getProfile(llclamp((S32)profile, 0, 7));
    const F32 infl = vc_sat(profileInfluence);
    auto vc_infl = [infl](F32 m) { return vc_lerp(1.f, m, infl); };

    const F32 dt = llclamp(input.mDeltaTime, 0.0005f, 0.25f);

    // ---- ground-truth speed metric (replaces MV estimation + AGC) ---------
    const F32 linRef = llmax((F32)refLinear, 0.01f);
    const F32 angRef = llmax((F32)refAngular, 1.f) * DEG_TO_RAD;

    // pan-plane flow equivalent: yaw+lateral => X, pitch+vertical => Y
    const F32 vx = input.mAngularVel.mV[VZ] / angRef + input.mLinearVel.mV[VY] / linRef;
    const F32 vy = input.mAngularVel.mV[VY] / angRef + input.mLinearVel.mV[VZ] / linRef;
    const F32 coherentRaw  = sqrtf(vx * vx + vy * vy);
    const F32 divergentRaw = fabsf(input.mLinearVel.mV[VX]) / linRef;
    const F32 rawSpd = llmin(coherentRaw + divergentRaw, REACT_MAX);

    // ---- framerate-independent smoothing -----------------------------------
    const F32 k = vc_retain(P.smoothing * vc_infl(Q.smoothMul) * smoothingTrim, dt);
    const F32 prevSpeed = mSpeed;
    mSpeed = vc_lerp(rawSpd, mSpeed, k);
    mVecX  = vc_lerp(vx, mVecX, k);
    mVecY  = vc_lerp(vy, mVecY, k);

    // ---- onset / settle envelopes ------------------------------------------
    const F32 accelN = (mSpeed - prevSpeed) * (DT_REF / dt);
    mOnsetEnv  = vc_sat(llmax(mOnsetEnv  * vc_retain(ONSET_DECAY, dt),  llmax(accelN, 0.f)  * ONSET_KICK));
    mSettleEnv = vc_sat(llmax(mSettleEnv * vc_retain(settleDecay, dt),  llmax(-accelN, 0.f) * SETTLE_KICK));

    // latched motion direction (settle bounce axis)
    const F32 coherent = sqrtf(mVecX * mVecX + mVecY * mVecY);
    if (coherent > 0.06f)
    {
        const F32 aDir = 1.f - expf(-dt / 0.15f);
        mLatchX = vc_lerp(mLatchX, mVecX / coherent, aDir);
        mLatchY = vc_lerp(mLatchY, mVecY / coherent, aDir);
    }

    // ---- phase accumulators (wrap-safe, pop-free on slider changes) --------
    const F32 divergent = llmax(mSpeed - coherent, 0.f);
    const F32 adv = dt * llmax((F32)timeSpeed, 0.f);
    const F32 gaitRate = P.walkRate * walkCadence * (1.f + vc_sat(divergent) * cadenceDrive);
    mPhaseXY        = vc_wrap(mPhaseXY        + adv * transFreq  * P.freqMul);
    mPhaseRoll      = vc_wrap(mPhaseRoll      + adv * rollFreq   * P.freqMul);
    mPhaseBreath    = vc_wrap(mPhaseBreath    + adv * breathFreq * P.freqMul);
    mPhaseGait      = vc_wrap(mPhaseGait      + adv * gaitRate);
    mPhaseSettle    = vc_wrap(mPhaseSettle    + adv * settleFreq);
    mPhaseRecompose = vc_wrap(mPhaseRecompose + adv / llmax((F32)recomposeInterval, 0.5f));

    // ---- reactive gains -----------------------------------------------------
    const F32 R = reactivity;
    const F32 eEnergy = P.energyGain * vc_infl(Q.energyMul) * R * energyTrim;
    const F32 eOnset  = P.onsetGain  * vc_infl(Q.onsetMul)  * R * onsetAmount  * 0.012f * FRAC_TO_RAD;
    const F32 eSettle = P.settleGain * vc_infl(Q.settleMul) * R * settleAmount * 0.010f * FRAC_TO_RAD;
    const F32 eDrag   = P.drag       * vc_infl(Q.dragMul)   * R * dragAmount   * FRAC_TO_RAD;
    const F32 eWalk   = P.walkCouple * vc_infl(Q.walkMul)   * R * gaitCoupling;
    const F32 eCalm   = P.motionCalm * vc_infl(Q.calmMul)   * motionCalmTrim;

    const F32 react = llmin(1.f + mSpeed * eEnergy, REACT_MAX);

    F32 mdirX = 0.f, mdirY = 0.f;
    if (coherent > 1e-4f) { mdirX = mVecX / coherent; mdirY = mVecY / coherent; }

    // direction-confidence gate (smoothed vec ramps from zero at onset)
    const F32 dirConf = vc_smoothstep(0.03f, 0.15f, coherent);
    const F32 whipX = mdirX * mOnsetEnv * eOnset * dirConf;
    const F32 whipY = mdirY * mOnsetEnv * eOnset * dirConf;
    const F32 dragX = -mdirX * coherent * eDrag;
    const F32 dragY = -mdirY * coherent * eDrag;
    const F32 settleW = mSettleEnv * eSettle;

    F32 sDirX = 0.f, sDirY = 1.f;   // settle axis fallback: vertical
    const F32 lLen = sqrtf(mLatchX * mLatchX + mLatchY * mLatchY);
    if (lLen > 0.05f) { sDirX = mLatchX / lLen; sDirY = mLatchY / lLen; }

    const F32 panLat = vc_sat(fabsf(mVecX) * 1.5f);

    F32 gait = 0.f;
    const F32 gaitDrive = (divergent * forwardWalkBias + coherent * lateralWalkBias) * eWalk;
    gait = vc_smoothstep(0.12f, 0.85f, gaitDrive);
    if (forceWalk) gait = llmax(gait, 1.f);

    // ---- idle tremor character ----------------------------------------------
    const F32 calm = vc_sat(mSpeed * eCalm);
    const F32 hi     = vc_lerp(1.4f, 0.2f, vc_sat(tremorDamping)) * P.hiMul * (1.f - calm);
    const F32 hiRoll = hi * (1.f - vc_sat(rollSmoothness) * 0.85f);
    const F32 latSmooth = vc_sat(lateralSmoothness + panLat * lateralPanBoost * (1.f - lateralSmoothness));
    const F32 hiX = hi * (1.f - latSmooth * 0.85f);
    const F32 sd = seed * 13.f;

    F32 sx;
    if ((S32)lateralMode == 1)      sx = vc_weave(mPhaseXY, sd);
    else if ((S32)lateralMode == 2) sx = vc_fbm(mPhaseXY + sd, hi);
    else                            sx = vc_fbmSmooth(mPhaseXY + sd, hiX);

    const F32 sy  = vc_fbm(mPhaseXY   + sd + 31.4f, hi);
    const F32 sr  = vc_fbm(mPhaseRoll + sd + 57.1f, hiRoll);
    const F32 sbz = vc_fbm(mPhaseBreath + sd + 83.9f, hi);

    const F32 mIdle = master * idleIntensity * P.ampMul;

    // per-axis reactive amplification of idle wander
    const F32 reactX = 1.f + (react - 1.f) * motionPan;
    const F32 reactY = 1.f + (react - 1.f) * motionTilt;
    const F32 reactR = 1.f + (react - 1.f) * 0.5f * motionRoll;   // roll half-weighted
    const F32 reactB = 1.f + (react - 1.f) * motionBreath;

    // idle wander is true camera rotation (deg settings -> radians)
    F32 yaw   = sx * panAmount  * DEG_TO_RAD * P.transMul * mIdle * reactX;
    F32 pitch = sy * tiltAmount * DEG_TO_RAD * P.transMul * mIdle * reactY;
    F32 roll  = sr * rollAmount * DEG_TO_RAD * P.rollMul  * mIdle * reactR;

    // breathing: true FOV pulse + coupled chest-rise translation
    const F32 breath = sbz * (breathAmount * 0.01f) * P.breathMul * mIdle * reactB;
    out.mFovMul = 1.f + breath;
    F32 liftZ = breath * breathLift * 0.15f;    // meters of chest rise

    // sparse recompose nudges (eased retarget of the framing center)
    if (recomposeAmount > 1e-4f)
    {
        F32 rcx, rcy;
        vc_recompose(mPhaseRecompose, sd, recomposeInterval, rcx, rcy);
        const F32 rcScale = 2.f * recomposeAmount * DEG_TO_RAD * P.transMul * master * idleIntensity;
        yaw   += rcx * rcScale;
        pitch += rcy * rcScale;
    }

    // additive reactive rotation, gated per axis
    yaw   += (whipX + dragX) * master * motionPan;
    pitch += (whipY + dragY) * master * motionTilt;

    // settle bounce along the axis of the motion that stopped
    const F32 wob = sinf(mPhaseSettle * F_TWO_PI) * settleW;
    yaw   += wob * sDirX * master * motionPan;
    pitch += wob * sDirY * master * motionTilt;
    roll  += wob * 1.5f * (0.4f + 0.6f * fabsf(sDirX)) * master * motionRoll;

    // ---- gait: real head translation (figure-8) -----------------------------
    F32 bobZ = 0.f, swayY = 0.f;
    if (gait > 0.001f)
    {
        const F32 foot   = mPhaseGait * F_TWO_PI;
        const F32 stride = mPhaseGait * F_PI;
        const F32 vbase  = -cosf(foot);
        const F32 vshape = vbase * (0.78f + 0.22f * vc_sat(-vbase));
        const F32 hbase  = sinf(stride);
        bobZ  = -vshape * stepBob * gait * master;          // up axis (Z)
        swayY =  hbase * lateralStep * gait * master;       // left axis (Y)
        roll += hbase * stepRoll * DEG_TO_RAD * gait * master;
        out.mFovMul += vbase * stepBob * 0.15f * gait;
    }

    // ---- outputs, with sanity clamps (no overscan needed: real camera) ------
    const F32 rotCap = 10.f * DEG_TO_RAD;
    out.mYaw   = llclamp(yaw,   -rotCap, rotCap);
    out.mPitch = llclamp(pitch, -rotCap, rotCap);
    out.mRoll  = llclamp(roll,  -rotCap, rotCap);
    out.mFovMul = llclamp(out.mFovMul, 0.8f, 1.25f);
    out.mPosOffset = LLVector3(0.f,
                               llclamp(swayY, -0.5f, 0.5f),
                               llclamp(bobZ + liftZ, -0.5f, 0.5f));
    return out;
}
