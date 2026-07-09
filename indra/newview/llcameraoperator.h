/**
 * @file llcameraoperator.h
 * @brief Procedural handheld camera operator for the flycam.
 *
 * Native port of the VirtualCinema_Handheld (VCHH) ReShade shader design:
 * OPERATOR STYLE x MOTION PROFILE personas, quintic value-noise idle motion
 * on wrapped phase accumulators (framerate-independent, no pops on slider
 * changes), a motion-reactive layer (onset whip, lead-lag drag, settle
 * bounce along the latched motion axis), figure-8 walking gait with cadence
 * drive, breathing with vertical lift coupling, and sparse recompose nudges.
 *
 * Unlike the shader, this runs on ground-truth camera motion (the flycam's
 * own per-frame deltas), so the estimation machinery of the original
 * (motion vectors, AGC, noise gates, cut detection heuristics) is not
 * needed. Output is applied to the *rendered* camera only -- the flycam's
 * authoritative position/rotation state is never mutated.
 *
 * $LicenseInfo:firstyear=2026&license=viewerlgpl$
 * Phoenix / Firestorm Viewer Source Code
 * $/LicenseInfo$
 */

#ifndef LL_LLCAMERAOPERATOR_H
#define LL_LLCAMERAOPERATOR_H

#include "stdtypes.h"
#include "v3math.h"

// Per-frame input: the flycam's real motion, camera-local.
struct LLCameraOperatorInput
{
    F32 mDeltaTime = 0.f;       // seconds, already clamped by caller
    // camera-local translation velocity, m/s (X=at/forward, Y=left, Z=up)
    LLVector3 mLinearVel = LLVector3::zero;
    // angular velocity, rad/s (X=roll, Y=pitch, Z=yaw) -- flycam rot deltas
    LLVector3 mAngularVel = LLVector3::zero;
};

// Per-frame output: offsets to apply to the rendered camera.
struct LLCameraOperatorOutput
{
    // camera-local translation offset, meters (X=at, Y=left, Z=up)
    LLVector3 mPosOffset = LLVector3::zero;
    // orientation wobble, radians
    F32 mRoll = 0.f;            // about at axis
    F32 mPitch = 0.f;           // about left axis
    F32 mYaw = 0.f;             // about up axis
    // multiplier on the vertical FOV (breathing)
    F32 mFovMul = 1.f;
};

class LLCameraOperator
{
public:
    static LLCameraOperator& instance();

    // Advance the simulation and produce this frame's camera offsets.
    // Reads all FlycamOperator* settings live.
    LLCameraOperatorOutput update(const LLCameraOperatorInput& input);

    // Clear all accumulated state (call on flycam toggle / reset so the
    // rig doesn't carry momentum across teleports or mode switches).
    void reset();

private:
    LLCameraOperator() = default;

    // ---- persistent state (the shader's FP32 state textures) ----
    // smoothed motion
    F32       mSpeed = 0.f;             // overall normalized speed (EMA)
    F32       mVecX = 0.f;              // smoothed pan-plane motion vector
    F32       mVecY = 0.f;
    // envelopes
    F32       mOnsetEnv = 0.f;
    F32       mSettleEnv = 0.f;
    // latched direction of the last real move (settle bounce axis)
    F32       mLatchX = 0.f;
    F32       mLatchY = 0.f;
    // wrapped phase accumulators
    F32       mPhaseXY = 0.f;
    F32       mPhaseRoll = 0.f;
    F32       mPhaseBreath = 0.f;
    F32       mPhaseGait = 0.f;
    F32       mPhaseSettle = 0.f;
    F32       mPhaseRecompose = 0.f;
};

#endif // LL_LLCAMERAOPERATOR_H
