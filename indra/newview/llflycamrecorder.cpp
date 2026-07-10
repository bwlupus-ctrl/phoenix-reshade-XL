/**
 * @file llflycamrecorder.cpp
 * @brief Camera path recorder: record the live camera, play it back, scrub it.
 *
 * $LicenseInfo:firstyear=2026&license=viewerlgpl$
 * Phoenix / Firestorm Viewer Source Code
 * $/LicenseInfo$
 */

#include "llviewerprecompiledheaders.h"

#include "llflycamrecorder.h"

#include <algorithm>
#include <cmath>

#include "llagent.h"
#include "llappviewer.h"        // gFrameIntervalSeconds
#include "llcameraoperator.h"
#include "llfile.h"
#include "llsdserialize.h"
#include "llsdutil.h"           // llsd::inArray
#include "llsdutil_math.h"      // ll_sd_from_vector3d, ll_sd_from_quaternion
#include "llviewercamera.h"
#include "llviewercontrol.h"    // gSavedSettings, LLCachedControl
#include "llviewerregion.h"
#include "m3math.h"

namespace
{
constexpr S32 TAKE_FORMAT_VERSION = 1;

// ---------------------------------------------------------------------------
// Raw quaternion math for squad interpolation. LLQuaternion's component
// constructor normalizes its input, which destroys the pure (w=0) log
// quaternions squad needs -- so the intermediates live in a plain struct.
// ---------------------------------------------------------------------------
struct FRQuat
{
    F32 x, y, z, w;
};

inline FRQuat fr_from(const LLQuaternion& q)
{
    return FRQuat{ q.mQ[VX], q.mQ[VY], q.mQ[VZ], q.mQ[VW] };
}

inline LLQuaternion fr_to(const FRQuat& q)
{
    LLQuaternion out;
    out.mQ[VX] = q.x;
    out.mQ[VY] = q.y;
    out.mQ[VZ] = q.z;
    out.mQ[VW] = q.w;
    out.normQuat();
    return out;
}

inline FRQuat fr_mul(const FRQuat& a, const FRQuat& b)
{
    return FRQuat{
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z };
}

inline FRQuat fr_conj(const FRQuat& q)          { return FRQuat{ -q.x, -q.y, -q.z, q.w }; }
inline FRQuat fr_scale(const FRQuat& q, F32 s)  { return FRQuat{ q.x * s, q.y * s, q.z * s, q.w * s }; }
inline FRQuat fr_add(const FRQuat& a, const FRQuat& b) { return FRQuat{ a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w }; }

// log of a unit quaternion -> pure quaternion (w = 0)
FRQuat fr_log(const FRQuat& q)
{
    const F32 w = llclamp(q.w, -1.f, 1.f);
    const F32 s = sqrtf(llmax(0.f, 1.f - w * w));
    if (s < 1e-6f)
    {
        return FRQuat{ 0.f, 0.f, 0.f, 0.f };
    }
    const F32 k = acosf(w) / s;
    return FRQuat{ q.x * k, q.y * k, q.z * k, 0.f };
}

// exp of a pure quaternion -> unit quaternion
FRQuat fr_exp(const FRQuat& q)
{
    const F32 theta = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z);
    if (theta < 1e-6f)
    {
        return FRQuat{ 0.f, 0.f, 0.f, 1.f };
    }
    const F32 k = sinf(theta) / theta;
    return FRQuat{ q.x * k, q.y * k, q.z * k, cosf(theta) };
}

FRQuat fr_slerp(const FRQuat& a, FRQuat b, F32 t)
{
    F32 dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (dot < 0.f)
    {
        b = fr_scale(b, -1.f);
        dot = -dot;
    }
    if (dot > 0.9995f)
    {
        // nearly parallel: normalized lerp
        FRQuat r = fr_add(fr_scale(a, 1.f - t), fr_scale(b, t));
        const F32 m = sqrtf(r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w);
        return (m > 1e-8f) ? fr_scale(r, 1.f / m) : a;
    }
    const F32 theta = acosf(llclamp(dot, -1.f, 1.f));
    const F32 inv_sin = 1.f / sinf(theta);
    return fr_add(fr_scale(a, sinf((1.f - t) * theta) * inv_sin),
                  fr_scale(b, sinf(t * theta) * inv_sin));
}

// squad inner control point for key q0 with neighbors qm1, qp1
FRQuat fr_squadInner(const FRQuat& qm1, const FRQuat& q0, const FRQuat& qp1)
{
    const FRQuat inv = fr_conj(q0);
    const FRQuat l0 = fr_log(fr_mul(inv, qm1));
    const FRQuat l1 = fr_log(fr_mul(inv, qp1));
    return fr_mul(q0, fr_exp(fr_scale(fr_add(l0, l1), -0.25f)));
}

// spherical cubic between q1..q2 with inner control points a, b
FRQuat fr_squad(const FRQuat& q1, const FRQuat& a, const FRQuat& b, const FRQuat& q2, F32 t)
{
    return fr_slerp(fr_slerp(q1, q2, t), fr_slerp(a, b, t), 2.f * t * (1.f - t));
}

// uniform Catmull-Rom (fine for near-uniform recorded samples; sparse
// hand-edited keys get gentle overshoot rather than corners)
inline F64 fr_catmullRom(F64 p0, F64 p1, F64 p2, F64 p3, F64 u)
{
    return 0.5 * ((2.0 * p1)
                  + (-p0 + p2) * u
                  + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u * u
                  + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u * u * u);
}

inline LLVector3d fr_catmullRom(const LLVector3d& p0, const LLVector3d& p1,
                                const LLVector3d& p2, const LLVector3d& p3, F64 u)
{
    return LLVector3d(fr_catmullRom(p0.mdV[VX], p1.mdV[VX], p2.mdV[VX], p3.mdV[VX], u),
                      fr_catmullRom(p0.mdV[VY], p1.mdV[VY], p2.mdV[VY], p3.mdV[VY], u),
                      fr_catmullRom(p0.mdV[VZ], p1.mdV[VZ], p2.mdV[VZ], p3.mdV[VZ], u));
}
} // anonymous namespace

// ---------------------------------------------------------------------------
LLFlycamRecorder& LLFlycamRecorder::instance()
{
    static LLFlycamRecorder sInstance;
    return sInstance;
}

bool LLFlycamRecorder::isPlaybackActive() const
{
    return (mState == STATE_PLAYING || mState == STATE_PAUSED) && !mKeys.empty();
}

F32 LLFlycamRecorder::getDuration() const
{
    return mKeys.empty() ? 0.f : mKeys.back().mTime;
}

// ---------------------------------------------------------------------------
// transport
// ---------------------------------------------------------------------------
void LLFlycamRecorder::startRecording()
{
    if (mState == STATE_RECORDING)
    {
        return;
    }
    // recording replaces the current take and releases any playback
    mState = STATE_RECORDING;
    mKeys.clear();
    mPlayhead = 0.f;
    mPlayDir = 1.f;
    mHavePrev = false;
    mRecordTimer.reset();
    sampleCamera(0.f);
    mStatus = "Recording...";
}

void LLFlycamRecorder::stopRecording()
{
    if (mState != STATE_RECORDING)
    {
        return;
    }
    // capture the final pose so the take ends exactly where the camera is
    sampleCamera(mRecordTimer.getElapsedTimeF32());
    mState = STATE_IDLE;
    mStatus = llformat("Recorded %.1f s (%d keys)", getDuration(), getNumKeyframes());
}

void LLFlycamRecorder::startPlayback()
{
    if (mState == STATE_RECORDING)
    {
        stopRecording();
    }
    if (mKeys.empty())
    {
        mStatus = "Nothing to play";
        return;
    }
    if (mPlayhead >= getDuration())
    {
        mPlayhead = 0.f;    // replay from the top when parked at the end
    }
    mPlayDir = 1.f;
    mHavePrev = false;
    LLCameraOperator::instance().reset();
    mState = STATE_PLAYING;
    mStatus = "Playing";
}

void LLFlycamRecorder::pausePlayback()
{
    if (mState == STATE_PLAYING)
    {
        mState = STATE_PAUSED;
        mStatus = "Paused";
    }
}

void LLFlycamRecorder::togglePlayback()
{
    if (mState == STATE_PLAYING)
    {
        pausePlayback();
    }
    else
    {
        startPlayback();
    }
}

void LLFlycamRecorder::stopPlayback()
{
    if (mState == STATE_PLAYING || mState == STATE_PAUSED)
    {
        mState = STATE_IDLE;
        mPlayhead = 0.f;
        mPlayDir = 1.f;
        mStatus = "Stopped";
    }
}

void LLFlycamRecorder::clear()
{
    if (mState == STATE_RECORDING || isPlaybackActive())
    {
        mState = STATE_IDLE;
    }
    mKeys.clear();
    mPlayhead = 0.f;
    mPlayDir = 1.f;
    mStatus = "Cleared";
}

void LLFlycamRecorder::seek(F32 time)
{
    if (mKeys.empty())
    {
        return;
    }
    mPlayhead = llclamp(time, 0.f, getDuration());
    mHavePrev = false;      // don't let the operator see the jump as motion
    if (mState == STATE_IDLE)
    {
        // scrubbing from idle previews the pose
        mState = STATE_PAUSED;
        mStatus = "Paused";
    }
}

// ---------------------------------------------------------------------------
// recording
// ---------------------------------------------------------------------------
void LLFlycamRecorder::onIdleFrame()
{
    if (mState != STATE_RECORDING)
    {
        return;
    }
    static LLCachedControl<F32> sample_rate(gSavedSettings, "FlycamRecSampleRate", 30.f);
    const F32 interval = 1.f / llclamp((F32)sample_rate, 1.f, 120.f);
    const F32 t = mRecordTimer.getElapsedTimeF32();
    if (mKeys.empty() || t - mKeys.back().mTime >= interval)
    {
        sampleCamera(t);
    }
}

void LLFlycamRecorder::sampleCamera(F32 time)
{
    LLViewerCamera* cam = LLViewerCamera::getInstance();
    Keyframe key;
    key.mTime = time;
    key.mPosGlobal = gAgent.getPosGlobalFromAgent(cam->getOrigin());
    key.mRot = cam->getQuaternion();
    key.mFov = cam->getView();
    if (!mKeys.empty())
    {
        // keep consecutive keys in the same hemisphere for interpolation
        const LLQuaternion& prev = mKeys.back().mRot;
        const F32 dot = prev.mQ[VX] * key.mRot.mQ[VX] + prev.mQ[VY] * key.mRot.mQ[VY]
                      + prev.mQ[VZ] * key.mRot.mQ[VZ] + prev.mQ[VW] * key.mRot.mQ[VW];
        if (dot < 0.f)
        {
            key.mRot = -key.mRot;
        }
        // guard against duplicate timestamps (zero-length segments)
        if (time <= mKeys.back().mTime)
        {
            return;
        }
    }
    mKeys.push_back(key);
}

void LLFlycamRecorder::canonicalizeRotations()
{
    for (size_t i = 1; i < mKeys.size(); ++i)
    {
        const LLQuaternion& prev = mKeys[i - 1].mRot;
        LLQuaternion& cur = mKeys[i].mRot;
        const F32 dot = prev.mQ[VX] * cur.mQ[VX] + prev.mQ[VY] * cur.mQ[VY]
                      + prev.mQ[VZ] * cur.mQ[VZ] + prev.mQ[VW] * cur.mQ[VW];
        if (dot < 0.f)
        {
            cur = -cur;
        }
    }
}

// ---------------------------------------------------------------------------
// playback
// ---------------------------------------------------------------------------
void LLFlycamRecorder::evalPose(F32 time, LLVector3d& pos, LLQuaternion& rot, F32& fov) const
{
    static LLCachedControl<bool> smooth(gSavedSettings, "FlycamRecSmooth", true);

    const size_t n = mKeys.size();
    if (n == 1)
    {
        pos = mKeys[0].mPosGlobal;
        rot = mKeys[0].mRot;
        fov = mKeys[0].mFov;
        return;
    }

    // bracketing segment [i1, i2]
    size_t i2 = 1;
    while (i2 < n - 1 && mKeys[i2].mTime < time)
    {
        ++i2;
    }
    const size_t i1 = i2 - 1;
    const Keyframe& k1 = mKeys[i1];
    const Keyframe& k2 = mKeys[i2];

    const F32 span = k2.mTime - k1.mTime;
    const F32 u = (span > 1e-6f) ? llclamp((time - k1.mTime) / span, 0.f, 1.f) : 0.f;

    fov = lerp(k1.mFov, k2.mFov, u);

    if (!smooth)
    {
        pos = lerp(k1.mPosGlobal, k2.mPosGlobal, (F64)u);
        rot = nlerp(u, k1.mRot, k2.mRot);
        return;
    }

    // smooth: Catmull-Rom position, squad orientation, using the clamped
    // neighbors k0 and k3
    const Keyframe& k0 = mKeys[(i1 > 0) ? i1 - 1 : 0];
    const Keyframe& k3 = mKeys[(i2 + 1 < n) ? i2 + 1 : n - 1];

    pos = fr_catmullRom(k0.mPosGlobal, k1.mPosGlobal, k2.mPosGlobal, k3.mPosGlobal, (F64)u);

    const FRQuat q0 = fr_from(k0.mRot);
    const FRQuat q1 = fr_from(k1.mRot);
    const FRQuat q2 = fr_from(k2.mRot);
    const FRQuat q3 = fr_from(k3.mRot);
    const FRQuat a = fr_squadInner(q0, q1, q2);
    const FRQuat b = fr_squadInner(q1, q2, q3);
    rot = fr_to(fr_squad(q1, a, b, q2, u));
}

void LLFlycamRecorder::updateCamera()
{
    static LLCachedControl<F32>  speed(gSavedSettings, "FlycamRecSpeed", 1.f);
    static LLCachedControl<S32>  loop_mode(gSavedSettings, "FlycamRecLoopMode", 0);
    static LLCachedControl<bool> use_operator(gSavedSettings, "FlycamRecUseOperator", false);

    if (mKeys.empty())
    {
        return;
    }

    const F32 dt = llclamp(gFrameIntervalSeconds.value(), 0.0005f, 0.25f);
    const F32 duration = getDuration();

    if (mState == STATE_PLAYING)
    {
        mPlayhead += dt * llclamp((F32)speed, 0.05f, 5.f) * mPlayDir;
        if (mPlayhead >= duration || mPlayhead <= 0.f)
        {
            switch ((S32)loop_mode)
            {
                case LOOP_REPEAT:
                    mPlayhead = (mPlayhead >= duration) ? mPlayhead - duration : mPlayhead + duration;
                    mHavePrev = false;      // the wrap is a cut, not motion
                    break;
                case LOOP_PINGPONG:
                    mPlayhead = (mPlayhead >= duration) ? 2.f * duration - mPlayhead : -mPlayhead;
                    mPlayDir = -mPlayDir;
                    break;
                default:                    // LOOP_HOLD_END: park on the end pose
                    mPlayhead = llclamp(mPlayhead, 0.f, duration);
                    mState = STATE_PAUSED;
                    mStatus = "Finished (holding last frame)";
                    break;
            }
        }
        mPlayhead = llclamp(mPlayhead, 0.f, duration);
    }

    LLVector3d pos_global;
    LLQuaternion rot;
    F32 fov;
    evalPose(mPlayhead, pos_global, rot, fov);

    LLVector3 out_pos = gAgent.getPosAgentFromGlobal(pos_global);
    LLQuaternion out_rot = rot;
    F32 fov_mul = 1.f;

    // ---- optional handheld texture on top (same idiom as LLCinematicCamera)
    if (use_operator && mState == STATE_PLAYING)
    {
        if (mHavePrev)
        {
            LLMatrix3 axes(out_rot);
            const LLVector3 world_vel = (out_pos - mPrevPosAgent) * (1.f / dt);
            LLQuaternion dq = out_rot * ~mPrevRot;
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
            mPrevPosAgent = out_pos;
            mPrevRot = out_rot;
            out_rot = LLQuaternion(wobble) * out_rot;
            LLMatrix3 out_axes(out_rot);
            out_pos += LLVector3(out_axes.mMatrix[0]) * op.mPosOffset.mV[VX]
                     + LLVector3(out_axes.mMatrix[1]) * op.mPosOffset.mV[VY]
                     + LLVector3(out_axes.mMatrix[2]) * op.mPosOffset.mV[VZ];
            fov_mul = op.mFovMul;
        }
        else
        {
            mPrevPosAgent = out_pos;
            mPrevRot = out_rot;
            mHavePrev = true;
        }
    }

    LLViewerCamera* cam = LLViewerCamera::getInstance();
    LLMatrix3 final_axes(out_rot);
    cam->setView(fov * fov_mul);
    cam->setOrigin(out_pos);
    cam->mXAxis = LLVector3(final_axes.mMatrix[0]);
    cam->mYAxis = LLVector3(final_axes.mMatrix[1]);
    cam->mZAxis = LLVector3(final_axes.mMatrix[2]);
}

// ---------------------------------------------------------------------------
// file I/O
// ---------------------------------------------------------------------------
bool LLFlycamRecorder::saveToFile(const std::string& filename)
{
    if (mKeys.empty())
    {
        mStatus = "Nothing to save";
        return false;
    }

    LLSD doc;
    doc["version"] = TAKE_FORMAT_VERSION;
    if (LLViewerRegion* region = gAgent.getRegion())
    {
        doc["region"] = region->getName();
    }
    LLSD keys = LLSD::emptyArray();
    for (const Keyframe& key : mKeys)
    {
        LLSD rec;
        rec["t"] = (LLSD::Real)key.mTime;
        rec["pos"] = ll_sd_from_vector3d(key.mPosGlobal);
        rec["rot"] = ll_sd_from_quaternion(key.mRot);
        rec["fov"] = (LLSD::Real)key.mFov;
        keys.append(rec);
    }
    doc["keyframes"] = keys;

    llofstream file(filename.c_str());
    if (!file.is_open())
    {
        mStatus = "Couldn't write " + filename;
        LL_WARNS() << mStatus << LL_ENDL;
        return false;
    }
    LLSDSerialize::toPrettyXML(doc, file);
    file.close();
    mStatus = llformat("Saved %d keys", getNumKeyframes());
    LL_INFOS() << "Flycam take saved to " << filename << LL_ENDL;
    return true;
}

bool LLFlycamRecorder::loadFromFile(const std::string& filename)
{
    llifstream file(filename.c_str());
    if (!file.is_open())
    {
        mStatus = "Couldn't open " + filename;
        LL_WARNS() << mStatus << LL_ENDL;
        return false;
    }
    LLSD doc;
    if (LLSDSerialize::fromXML(doc, file) == LLSDParser::PARSE_FAILURE
        || !doc.isMap() || !doc.has("keyframes"))
    {
        mStatus = "Not a flycam take file";
        LL_WARNS() << mStatus << ": " << filename << LL_ENDL;
        return false;
    }
    file.close();

    std::vector<Keyframe> keys;
    const LLSD keyframes = doc["keyframes"];    // named: keeps the array alive for the loop
    for (const LLSD& rec : llsd::inArray(keyframes))
    {
        Keyframe key;
        key.mTime = (F32)rec["t"].asReal();
        key.mPosGlobal = ll_vector3d_from_sd(rec["pos"]);
        key.mRot = ll_quaternion_from_sd(rec["rot"]);
        key.mRot.normQuat();
        key.mFov = (F32)rec["fov"].asReal();
        keys.push_back(key);
    }
    if (keys.empty())
    {
        mStatus = "Take file has no keyframes";
        return false;
    }
    // tolerate hand-edited files: sort by time, drop duplicate timestamps
    std::stable_sort(keys.begin(), keys.end(),
                     [](const Keyframe& a, const Keyframe& b) { return a.mTime < b.mTime; });
    std::vector<Keyframe> unique_keys;
    unique_keys.reserve(keys.size());
    for (const Keyframe& key : keys)
    {
        if (unique_keys.empty() || key.mTime > unique_keys.back().mTime)
        {
            unique_keys.push_back(key);
        }
    }

    mState = STATE_IDLE;
    mKeys.swap(unique_keys);
    mPlayhead = 0.f;
    mPlayDir = 1.f;
    canonicalizeRotations();
    mStatus = llformat("Loaded %d keys (%.1f s)", getNumKeyframes(), getDuration());
    LL_INFOS() << "Flycam take loaded from " << filename << LL_ENDL;
    return true;
}
