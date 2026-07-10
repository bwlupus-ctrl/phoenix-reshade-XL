/**
 * @file llflycamrecorder.h
 * @brief Camera path recorder: record the live camera, play it back, scrub it.
 *
 * Records the final render camera (position, full orientation, FOV) each
 * frame at a configurable sample rate -- regardless of what is driving it
 * (joystick flycam, mouse orbit, cinematic camera). Playback re-takes the
 * camera (a branch in the idle camera dispatch, like LLCinematicCamera) and
 * evaluates the recorded path with either linear interpolation or smooth
 * splines (Catmull-Rom on position/FOV, squad on orientation), so sparse
 * hand-edited keyframes still produce fluid motion.
 *
 * Takes are saved/loaded as a single hand-editable LLSD XML document:
 *   { version, region, keyframes: [ { t, pos(global), rot(x,y,z,w), fov } ] }
 *
 * Playback supports speed scaling, loop / ping-pong, pause, and scrubbing
 * to any time (seeking while idle previews the pose by entering PAUSED).
 * The procedural handheld operator (LLCameraOperator) can optionally be
 * layered on top of playback, fed with velocities derived from the path --
 * record a clean path with the operator off, re-texture it on playback.
 *
 * $LicenseInfo:firstyear=2026&license=viewerlgpl$
 * Phoenix / Firestorm Viewer Source Code
 * $/LicenseInfo$
 */

#ifndef LL_LLFLYCAMRECORDER_H
#define LL_LLFLYCAMRECORDER_H

#include <string>
#include <vector>

#include "stdtypes.h"
#include "lltimer.h"
#include "llquaternion.h"
#include "v3dmath.h"
#include "v3math.h"

class LLFlycamRecorder
{
public:
    enum EState
    {
        STATE_IDLE      = 0,
        STATE_RECORDING = 1,
        STATE_PLAYING   = 2,
        STATE_PAUSED    = 3,    // owns the camera, holding the playhead pose
    };

    enum ELoopMode
    {
        LOOP_HOLD_END   = 0,    // pause on the last frame (no jarring cut)
        LOOP_REPEAT     = 1,
        LOOP_PINGPONG   = 2,
    };

    struct Keyframe
    {
        F32          mTime = 0.f;       // seconds from take start
        LLVector3d   mPosGlobal;        // camera origin, global coordinates
        LLQuaternion mRot;              // camera orientation, world frame
        F32          mFov = 1.f;        // vertical FOV, radians
    };

    static LLFlycamRecorder& instance();

    // ---- frame hooks (called from the idle loop in llappviewer.cpp) ----
    // True when playback owns the render camera this frame.
    bool isPlaybackActive() const;
    // Advance playback and write the render camera. Call from the idle
    // camera dispatch INSTEAD of gAgentCamera.updateCamera() when active.
    void updateCamera();
    // Sample the (already updated) render camera if recording. Call once
    // per frame after the camera dispatch.
    void onIdleFrame();

    // ---- transport ----
    void startRecording();      // clears the current take and records fresh
    void stopRecording();
    void startPlayback();       // resumes from the playhead
    void pausePlayback();
    void togglePlayback();      // play/pause convenience for the UI
    void stopPlayback();        // releases the camera, playhead back to 0
    void clear();               // drop all keyframes

    // ---- scrubbing ----
    // Move the playhead (seconds, clamped). If idle with data, enters
    // PAUSED so the camera jumps there for preview.
    void seek(F32 time);

    // ---- file I/O (LLSD XML) ----
    bool saveToFile(const std::string& filename);
    bool loadFromFile(const std::string& filename);

    // ---- introspection (for the floater) ----
    EState             getState() const        { return mState; }
    F32                getPlayhead() const     { return mPlayhead; }
    F32                getDuration() const;
    S32                getNumKeyframes() const { return (S32)mKeys.size(); }
    const std::string& getStatus() const       { return mStatus; }

private:
    LLFlycamRecorder() = default;

    void sampleCamera(F32 time);
    // Evaluate the path pose at a time (clamped to the take's range).
    void evalPose(F32 time, LLVector3d& pos, LLQuaternion& rot, F32& fov) const;
    // Flip quaternion signs so consecutive keys share a hemisphere
    // (keeps nlerp/squad from taking the long way around).
    void canonicalizeRotations();

    // ---- state ----
    EState               mState = STATE_IDLE;
    std::vector<Keyframe> mKeys;
    F32                  mPlayhead = 0.f;
    F32                  mPlayDir = 1.f;        // ping-pong direction
    LLTimer              mRecordTimer;
    std::string          mStatus;

    // previous playback pose, for feeding the handheld operator
    bool                 mHavePrev = false;
    LLVector3            mPrevPosAgent;
    LLQuaternion         mPrevRot;
};

#endif // LL_LLFLYCAMRECORDER_H
