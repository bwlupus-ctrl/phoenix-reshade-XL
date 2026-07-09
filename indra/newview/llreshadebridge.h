/**
 * @file llreshadebridge.h
 * @brief In-process bridge that exposes the viewer's G-buffer and camera data
 *        to an injected ReShade 6.x runtime as an add-on.
 *
 * $LicenseInfo:firstyear=2026&license=viewerlgpl$
 * Phoenix / Firestorm Viewer Source Code
 * $/LicenseInfo$
 */

#ifndef LL_LLRESHADEBRIDGE_H
#define LL_LLRESHADEBRIDGE_H

#include "stdtypes.h"

// Compile the ReShade add-on glue only when the ReShade add-on SDK header
// (reshade.hpp) has been vendored into the tree and wired up in CMake.
// Until then the viewer-side gather still builds and runs (as a no-op sink),
// so nothing here can break a normal build.
#ifndef LL_RESHADE_ADDON
#define LL_RESHADE_ADDON 0
#endif

// Snapshot of everything ReShade needs from the viewer for a single frame.
// GL texture handles are raw OpenGL names (LLRenderTarget::getTexture/getDepth).
// Matrices are stored row-major exactly as LLMatrix4 lays them out (mMatrix[r][c]).
struct LLReShadeFrameData
{
    // --- Camera scalars ---
    F32 mNear = 0.f;            // near clip plane, meters
    F32 mFar  = 0.f;            // far clip plane, meters (tracks RenderFarClip)
    F32 mFovY = 0.f;            // vertical field of view, radians
    F32 mAspect = 0.f;          // width / height

    // --- Camera transform (world space), lets shaders reconstruct without inverses ---
    F32 mOrigin[3]   = { 0.f, 0.f, 0.f };   // camera world position
    F32 mAtAxis[3]   = { 0.f, 0.f, 0.f };   // forward
    F32 mLeftAxis[3] = { 0.f, 0.f, 0.f };   // left
    F32 mUpAxis[3]   = { 0.f, 0.f, 0.f };   // up

    // --- Matrices (16 floats each, row-major as LLMatrix4) ---
    F32 mView[16] = { 0.f };    // modelview
    F32 mProj[16] = { 0.f };    // projection

    // --- Render target dimensions ---
    U32 mWidth  = 0;
    U32 mHeight = 0;

    // --- GL texture handles (0 == not available this frame) ---
    U32 mTexColorHDR = 0;       // scene HDR color (pre-tonemap), pipeline "screen"
    U32 mTexDepth    = 0;       // depth buffer (shared by deferredScreen/screen)
    U32 mTexAlbedo   = 0;       // deferredScreen attachment 0
    U32 mTexORM      = 0;       // deferredScreen attachment 1 (occlusion/roughness/metallic)
    U32 mTexNormals  = 0;       // deferredScreen attachment 2 (encoded world normals)
    U32 mTexEmissive = 0;       // deferredScreen attachment 3 (optional; 0 if disabled)

    bool mValid = false;        // false if the pipeline had no usable RT this frame
};

// Singleton bridge. init()/shutdown() manage the ReShade add-on registration;
// gatherFrame() is called once per frame from the display loop, right after the
// scene is finalized and before the UI is composited.
class LLReShadeBridge
{
public:
    static LLReShadeBridge& instance();

    void init();        // register add-on + events (no-op unless LL_RESHADE_ADDON)
    void shutdown();    // unregister

    // Capture the current frame's camera + G-buffer state. Cheap: reads existing
    // GL handles and camera state, no GPU work. Safe to call every frame.
    void gatherFrame();

    const LLReShadeFrameData& getFrameData() const { return mFrame; }
    bool isEnabled() const { return mEnabled; }

private:
    LLReShadeBridge() = default;

    LLReShadeFrameData mFrame;
    bool mEnabled = false;   // true once a ReShade runtime is present and we're registered
};

#endif // LL_LLRESHADEBRIDGE_H
