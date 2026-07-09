/**
 * @file llcinematiccamera.cpp
 * @brief Automated cinematic camera: bone-lock (GoPro) and motion patterns.
 *
 * $LicenseInfo:firstyear=2026&license=viewerlgpl$
 * Phoenix / Firestorm Viewer Source Code
 * $/LicenseInfo$
 */

#include "llviewerprecompiledheaders.h"

#include "llcinematiccamera.h"

#include <cmath>

#include "llcameraoperator.h"
#include "llappviewer.h"            // gFrameIntervalSeconds
#include "lljoint.h"
#include "llmath.h"
#include "llselectmgr.h"
#include "llviewercamera.h"
#include "llviewercontrol.h"        // gSavedSettings, LLCachedControl
#include "llviewerobject.h"
#include "llvoavatar.h"
#include "llvoavatarself.h"         // gAgentAvatarp, isAgentAvatarValid()
#include "m3math.h"

namespace
{
constexpr F32 PHASE_WRAP = 4096.f;  // seconds of pattern clock before wrap

inline F32 cc_frac(F32 x)               { return x - floorf(x); }
inline F32 cc_lerp(F32 a, F32 b, F32 u) { return a + (b - a) * u; }

// small periodic value noise (same construction as the camera operator)
F32 cc_hash(F32 p)
{
    p = p - floorf(p / 256.f) * 256.f;
    p = cc_frac(p * 0.1031f);
    p *= p + 33.33f;
    p *= p + p;
    return cc_frac(p);
}

F32 cc_noise(F32 x)
{
    F32 i = floorf(x);
    F32 f = cc_frac(x);
    F32 u = f * f * f * (f * (f * 6.f - 15.f) + 10.f);
    return cc_lerp(cc_hash(i), cc_hash(i + 1.f), u) - 0.5f;
}

F32 cc_fbm(F32 x)
{
    return (cc_noise(x) + 0.5f * cc_noise(x * 2.f + 17.3f) + 0.25f * cc_noise(x * 4.f + 41.9f)) * 1.4f;
}

// build a level (Z-up) look-at orientation: X=at, Y=left, Z=up
LLQuaternion cc_lookAt(const LLVector3& from, const LLVector3& to)
{
    LLVector3 at = to - from;
    if (at.magVecSquared() < 1e-8f)
    {
        return LLQuaternion();
    }
    at.normVec();
    LLVector3 up_world(0.f, 0.f, 1.f);
    LLVector3 left = up_world % at;     // cross: y = z x x
    if (left.magVecSquared() < 1e-6f)   // looking straight up/down
    {
        left = LLVector3(0.f, 1.f, 0.f);
    }
    left.normVec();
    LLVector3 up = at % left;
    LLMatrix3 mat;
    mat.setRows(at, left, up);
    return LLQuaternion(mat);
}

// strip roll from an orientation, keeping its at-axis (horizon lock)
LLQuaternion cc_levelHorizon(const LLQuaternion& q)
{
    LLMatrix3 m(q);
    LLVector3 at(m.mMatrix[0]);
    LLVector3 up_world(0.f, 0.f, 1.f);
    LLVector3 left = up_world % at;
    if (left.magVecSquared() < 1e-6f)
    {
        return q;   // degenerate (looking straight up/down): keep as-is
    }
    left.normVec();
    LLVector3 up = at % left;
    LLMatrix3 level;
    level.setRows(at, left, up);
    return LLQuaternion(level);
}
} // anonymous namespace

// ---------------------------------------------------------------------------
LLCinematicCamera& LLCinematicCamera::instance()
{
    static LLCinematicCamera sInstance;
    return sInstance;
}

bool LLCinematicCamera::isActive() const
{
    static LLCachedControl<bool> enabled(gSavedSettings, "CinematicCamEnabled", false);
    static LLCachedControl<S32>  mode(gSavedSettings, "CinematicCamMode", 1);
    if (!enabled || (S32)mode <= MODE_OFF || (S32)mode > MODE_CRANE)
    {
        return false;
    }
    return resolveTarget() != nullptr;
}

LLVOAvatar* LLCinematicCamera::resolveTarget() const
{
    static LLCachedControl<bool> use_selected(gSavedSettings, "CinematicCamUseSelected", false);
    if (use_selected)
    {
        LLViewerObject* obj = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
        if (obj)
        {
            LLVOAvatar* av = obj->getAvatar();  // avatar itself or attachment parent
            if (av && !av->isDead())
            {
                return av;
            }
        }
        // fall through to self so the camera doesn't drop out mid-shot
    }
    return isAgentAvatarValid() ? (LLVOAvatar*)gAgentAvatarp : nullptr;
}

// ---------------------------------------------------------------------------
// pattern generators (agent region coordinates, Z up)
// ---------------------------------------------------------------------------
void LLCinematicCamera::patternBoneLock(LLVOAvatar* av, F32 /*phase*/,
                                        LLVector3& pos, LLQuaternion& rot, bool& have_rot)
{
    static LLCachedControl<std::string> joint_name(gSavedSettings, "CinematicCamJoint", std::string("mHead"));
    static LLCachedControl<F32> off_fwd(gSavedSettings, "CinematicCamBoneOffsetForward", 0.1f);
    static LLCachedControl<F32> off_left(gSavedSettings, "CinematicCamBoneOffsetLeft", 0.f);
    static LLCachedControl<F32> off_up(gSavedSettings, "CinematicCamBoneOffsetUp", 0.05f);
    static LLCachedControl<F32> aim_yaw(gSavedSettings, "CinematicCamBoneAimYaw", 0.f);
    static LLCachedControl<F32> aim_pitch(gSavedSettings, "CinematicCamBoneAimPitch", 0.f);
    static LLCachedControl<F32> aim_roll(gSavedSettings, "CinematicCamBoneAimRoll", 0.f);
    static LLCachedControl<bool> horizon(gSavedSettings, "CinematicCamBoneHorizonLock", false);

    LLJoint* joint = av->getJoint(std::string(joint_name));
    if (!joint)
    {
        joint = av->getJoint("mHead");
    }
    if (!joint)
    {
        pos = av->getPositionAgent() + LLVector3(0.f, 0.f, 1.f);
        have_rot = false;
        return;
    }

    LLQuaternion jrot = joint->getWorldRotation();
    // aim trim (degrees) applied in the joint's local frame
    LLQuaternion trim;
    trim.setEulerAngles(aim_roll * DEG_TO_RAD, aim_pitch * DEG_TO_RAD, aim_yaw * DEG_TO_RAD);
    rot = trim * jrot;
    if (horizon)
    {
        rot = cc_levelHorizon(rot);
    }
    have_rot = true;

    // local mount offset in the (trimmed) camera frame: X=at, Y=left, Z=up
    LLVector3 offset = LLVector3(off_fwd, off_left, off_up) * rot;
    pos = joint->getWorldPosition() + offset;
}

LLVector3 LLCinematicCamera::patternOrbit(const LLVector3& center, F32 phase)
{
    static LLCachedControl<F32> radius(gSavedSettings, "CinematicCamOrbitRadius", 3.f);
    static LLCachedControl<F32> speed(gSavedSettings, "CinematicCamOrbitSpeed", 20.f);   // deg/s
    static LLCachedControl<F32> height(gSavedSettings, "CinematicCamOrbitHeight", 0.5f);
    static LLCachedControl<F32> bob(gSavedSettings, "CinematicCamOrbitBob", 0.f);

    const F32 a = phase * speed * DEG_TO_RAD;
    return center + LLVector3(cosf(a) * radius,
                              sinf(a) * radius,
                              height + bob * sinf(a * 2.7f));
}

LLVector3 LLCinematicCamera::patternHover(const LLVector3& center, F32 phase)
{
    static LLCachedControl<F32> distance(gSavedSettings, "CinematicCamHoverDistance", 2.5f);
    static LLCachedControl<F32> wander(gSavedSettings, "CinematicCamHoverWander", 1.f);
    static LLCachedControl<F32> speed(gSavedSettings, "CinematicCamHoverSpeed", 0.35f);
    static LLCachedControl<F32> height(gSavedSettings, "CinematicCamHoverHeight", 0.6f);

    // a fly: azimuth drifts on noise, elevation and range breathe on
    // decorrelated noise, plus fine jitter
    const F32 t = phase * speed;
    const F32 az = t * 0.9f + cc_fbm(t * 0.7f) * 3.f;
    const F32 el = cc_fbm(t * 0.55f + 31.7f) * 0.6f;
    const F32 rr = distance * (1.f + 0.25f * cc_fbm(t * 0.8f + 57.1f) * wander);
    LLVector3 p(cosf(az) * cosf(el) * rr,
                sinf(az) * cosf(el) * rr,
                height + sinf(el) * rr * 0.5f);
    // fine wing-jitter
    p += LLVector3(cc_fbm(t * 5.3f + 11.f), cc_fbm(t * 5.9f + 23.f), cc_fbm(t * 6.7f + 47.f)) * 0.06f * wander;
    return center + p;
}

LLVector3 LLCinematicCamera::patternSweep(const LLVector3& center, F32 phase)
{
    static LLCachedControl<F32> length(gSavedSettings, "CinematicCamSweepLength", 8.f);
    static LLCachedControl<F32> distance(gSavedSettings, "CinematicCamSweepDistance", 3.f);
    static LLCachedControl<F32> speed(gSavedSettings, "CinematicCamSweepSpeed", 1.f);    // m/s
    static LLCachedControl<F32> heading(gSavedSettings, "CinematicCamSweepHeading", 0.f); // deg
    static LLCachedControl<F32> height(gSavedSettings, "CinematicCamSweepHeight", 0.5f);
    static LLCachedControl<bool> pingpong(gSavedSettings, "CinematicCamSweepPingPong", true);

    const F32 h = heading * DEG_TO_RAD;
    const LLVector3 dir(cosf(h), sinf(h), 0.f);         // travel direction
    const LLVector3 perp(-sinf(h), cosf(h), 0.f);       // offset from subject

    const F32 len = llmax((F32)length, 0.1f);
    F32 s = phase * llmax((F32)speed, 0.01f) / len;     // path cycles
    F32 t;
    if (pingpong)
    {
        const F32 c = cc_frac(s * 0.5f) * 2.f;          // 0..2
        t = (c < 1.f) ? c : 2.f - c;                    // triangle 0..1..0
    }
    else
    {
        t = cc_frac(s);
    }
    return center + perp * distance + dir * ((t - 0.5f) * len) + LLVector3(0.f, 0.f, height);
}

LLVector3 LLCinematicCamera::patternCrane(const LLVector3& center, F32 phase)
{
    static LLCachedControl<F32> radius(gSavedSettings, "CinematicCamCraneRadius", 4.f);
    static LLCachedControl<F32> speed(gSavedSettings, "CinematicCamCraneSpeed", 8.f);    // deg/s
    static LLCachedControl<F32> min_h(gSavedSettings, "CinematicCamCraneMinHeight", 0.2f);
    static LLCachedControl<F32> max_h(gSavedSettings, "CinematicCamCraneMaxHeight", 4.f);
    static LLCachedControl<F32> rise_period(gSavedSettings, "CinematicCamCraneRisePeriod", 14.f);

    const F32 a = phase * speed * DEG_TO_RAD;
    const F32 u = 0.5f + 0.5f * sinf(phase * F_TWO_PI / llmax((F32)rise_period, 1.f));
    return center + LLVector3(cosf(a) * radius, sinf(a) * radius, cc_lerp(min_h, max_h, u));
}

// ---------------------------------------------------------------------------
void LLCinematicCamera::updateCamera()
{
    static LLCachedControl<S32>  mode(gSavedSettings, "CinematicCamMode", 1);
    static LLCachedControl<F32>  smoothing(gSavedSettings, "CinematicCamSmoothing", 0.35f);   // seconds
    static LLCachedControl<bool> look_at_head(gSavedSettings, "CinematicCamLookAtHead", true);
    static LLCachedControl<bool> use_operator(gSavedSettings, "CinematicCamUseOperator", false);

    LLVOAvatar* av = resolveTarget();
    if (!av)
    {
        return;
    }

    F32 dt = llclamp(gFrameIntervalSeconds.value(), 0.0005f, 0.25f);
    mPhase += dt;
    if (mPhase > PHASE_WRAP)
    {
        mPhase -= PHASE_WRAP;
    }

    // the point patterns frame: head when available, else chest height
    LLVector3 focus = av->getPositionAgent() + LLVector3(0.f, 0.f, 1.f);
    if (look_at_head)
    {
        if (LLJoint* head = av->getJoint("mHead"))
        {
            focus = head->getWorldPosition();
        }
    }
    const LLVector3 center = av->getPositionAgent();

    LLVector3 pos;
    LLQuaternion rot;
    bool have_rot = false;

    switch ((S32)mode)
    {
        case MODE_BONE_LOCK: patternBoneLock(av, mPhase, pos, rot, have_rot); break;
        case MODE_ORBIT:     pos = patternOrbit(center, mPhase); break;
        case MODE_FLY_HOVER: pos = patternHover(center, mPhase); break;
        case MODE_SWEEP:     pos = patternSweep(center, mPhase); break;
        case MODE_CRANE:     pos = patternCrane(center, mPhase); break;
        default:             return;
    }

    if (!have_rot)
    {
        rot = cc_lookAt(pos, focus);
    }

    // ---- temporal smoothing (one-pole, framerate-independent) -------------
    const F32 tau = llmax((F32)smoothing, 0.f);
    if (!mHavePose || tau < 1e-3f)
    {
        mSmPos = pos;
        mSmRot = rot;
        mHavePose = true;
    }
    else
    {
        const F32 alpha = 1.f - expf(-dt / tau);
        mSmPos = mSmPos + (pos - mSmPos) * alpha;
        mSmRot = nlerp(alpha, mSmRot, rot);
    }

    LLVector3 out_pos = mSmPos;
    LLQuaternion out_rot = mSmRot;
    F32 fov_mul = 1.f;

    // ---- optional handheld texture on top ---------------------------------
    if (use_operator)
    {
        if (!mWasActive)
        {
            LLCameraOperator::instance().reset();
        }
        LLMatrix3 axes(mSmRot);
        const LLVector3 world_vel = (mSmPos - mPrevPos) * (1.f / dt);
        // angular velocity from the frame-to-frame rotation delta
        LLQuaternion dq = mSmRot * ~mPrevRot;
        F32 d_roll, d_pitch, d_yaw;
        LLMatrix3(dq).getEulerAngles(&d_roll, &d_pitch, &d_yaw);

        LLCameraOperatorInput opin;
        opin.mDeltaTime = dt;
        opin.mLinearVel = LLVector3(world_vel * LLVector3(axes.mMatrix[0]),
                                    world_vel * LLVector3(axes.mMatrix[1]),
                                    world_vel * LLVector3(axes.mMatrix[2]));
        opin.mAngularVel = LLVector3(d_roll, d_pitch, d_yaw) * (1.f / dt);

        const LLCameraOperatorOutput op = LLCameraOperator::instance().update(opin);
        LLMatrix3 wobble(op.mRoll, op.mPitch, op.mYaw);
        out_rot = LLQuaternion(wobble) * out_rot;
        LLMatrix3 out_axes(out_rot);
        out_pos += LLVector3(out_axes.mMatrix[0]) * op.mPosOffset.mV[VX]
                 + LLVector3(out_axes.mMatrix[1]) * op.mPosOffset.mV[VY]
                 + LLVector3(out_axes.mMatrix[2]) * op.mPosOffset.mV[VZ];
        fov_mul = op.mFovMul;
    }

    mPrevPos = mSmPos;
    mPrevRot = mSmRot;
    mWasActive = true;

    // ---- write the render camera -------------------------------------------
    LLViewerCamera* cam = LLViewerCamera::getInstance();
    LLMatrix3 final_axes(out_rot);
    cam->setView(cam->getDefaultFOV() * fov_mul);
    cam->setOrigin(out_pos);
    cam->mXAxis = LLVector3(final_axes.mMatrix[0]);
    cam->mYAxis = LLVector3(final_axes.mMatrix[1]);
    cam->mZAxis = LLVector3(final_axes.mMatrix[2]);
}
