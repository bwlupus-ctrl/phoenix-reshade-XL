/**
 * @file llcinematiccamera.h
 * @brief Automated cinematic camera: bone-lock (GoPro) and motion patterns.
 *
 * Takes over the render camera (highest-priority branch in the idle camera
 * dispatch) and drives it from one of several cinematic sources:
 *
 *   Bone Lock  -- rigidly attach the camera to an avatar skeleton joint,
 *                 like a GoPro strapped to that bone, with local position
 *                 and aim offsets, optional horizon lock and smoothing.
 *   Orbit      -- circle the target at a radius/height with optional bob.
 *   Fly Hover  -- drone/fly-like wander around the target on fBm noise,
 *                 always framing the target.
 *   Sweep      -- linear dolly pass by the target (ping-pong or loop).
 *   Crane      -- slow orbit while the height eases between two levels.
 *
 * The target is your own avatar, or the selected avatar when
 * CinematicCamUseSelected is set. Output can optionally be routed through
 * LLCameraOperator for the handheld texture on top of the pattern.
 *
 * $LicenseInfo:firstyear=2026&license=viewerlgpl$
 * Phoenix / Firestorm Viewer Source Code
 * $/LicenseInfo$
 */

#ifndef LL_LLCINEMATICCAMERA_H
#define LL_LLCINEMATICCAMERA_H

#include "stdtypes.h"
#include "v3math.h"
#include "llquaternion.h"

class LLVOAvatar;

class LLCinematicCamera
{
public:
    enum EMode
    {
        MODE_OFF        = 0,
        MODE_BONE_LOCK  = 1,
        MODE_ORBIT      = 2,
        MODE_FLY_HOVER  = 3,
        MODE_SWEEP      = 4,
        MODE_CRANE      = 5,
    };

    static LLCinematicCamera& instance();

    // True when the system should own the render camera this frame.
    bool isActive() const;

    // Compute and write this frame's camera. Call from the idle camera
    // dispatch INSTEAD of gAgentCamera.updateCamera() when isActive().
    void updateCamera();

private:
    LLCinematicCamera() = default;

    LLVOAvatar* resolveTarget() const;

    // pattern generators: produce a desired camera position and the point
    // to frame, in agent region coordinates
    void patternBoneLock(LLVOAvatar* av, F32 phase, LLVector3& pos, LLQuaternion& rot, bool& have_rot);
    LLVector3 patternOrbit(const LLVector3& center, F32 phase);
    LLVector3 patternHover(const LLVector3& center, F32 phase);
    LLVector3 patternSweep(const LLVector3& center, F32 phase);
    LLVector3 patternCrane(const LLVector3& center, F32 phase);

    // ---- state ----
    bool        mWasActive = false;
    F32         mPhase = 0.f;           // wrapped pattern clock, seconds*speed
    F32         mPrevTime = 0.f;
    // smoothed pose
    bool        mHavePose = false;
    LLVector3   mSmPos = LLVector3::zero;
    LLQuaternion mSmRot;
    // for feeding the camera operator with velocities
    LLVector3   mPrevPos = LLVector3::zero;
    LLQuaternion mPrevRot;
};

#endif // LL_LLCINEMATICCAMERA_H
