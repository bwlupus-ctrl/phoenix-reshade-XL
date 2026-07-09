/**
 * @file llreshadebridge.cpp
 * @brief In-process bridge exposing the viewer G-buffer + camera to ReShade 6.x.
 *
 * $LicenseInfo:firstyear=2026&license=viewerlgpl$
 * Phoenix / Firestorm Viewer Source Code
 * $/LicenseInfo$
 */

#include "llviewerprecompiledheaders.h"

#include "llreshadebridge.h"

#include <cstring>              // memcpy

#include "pipeline.h"          // gPipeline, RenderTargetPack, LLRenderTarget
#include "llviewercamera.h"    // LLViewerCamera (matrices, near/far/fov, world frame)

#if LL_RESHADE_ADDON
// Vendored ReShade add-on SDK header (Apache-2.0). Only pulled in once the
// header exists in the tree and CMake adds its include dir. See notes at bottom.
#include "reshade.hpp"
#endif

// -----------------------------------------------------------------------------
LLReShadeBridge& LLReShadeBridge::instance()
{
    static LLReShadeBridge sInstance;
    return sInstance;
}

// -----------------------------------------------------------------------------
// gatherFrame -- pull this frame's camera + G-buffer state into mFrame.
// Pure CPU reads of already-resident GL handles and camera state; no GPU work.
// -----------------------------------------------------------------------------
void LLReShadeBridge::gatherFrame()
{
    LLReShadeFrameData& f = mFrame;
    f = LLReShadeFrameData();   // reset; mValid defaults false

    LLViewerCamera* cam = LLViewerCamera::getInstance();
    if (!cam)
    {
        return;
    }

    // Camera scalars
    f.mNear   = cam->getNear();
    f.mFar    = cam->getFar();
    f.mFovY   = cam->getView();     // vertical FOV, radians
    f.mAspect = cam->getAspect();

    // World-space camera frame (origin + basis) -- lets shaders reconstruct
    // world position from depth without needing inverse matrices marshaled.
    const LLVector3& o    = cam->getOrigin();
    const LLVector3& at   = cam->getAtAxis();
    const LLVector3& left = cam->getLeftAxis();
    const LLVector3& up   = cam->getUpAxis();
    for (U32 i = 0; i < 3; ++i)
    {
        f.mOrigin[i]   = o.mV[i];
        f.mAtAxis[i]   = at.mV[i];
        f.mLeftAxis[i] = left.mV[i];
        f.mUpAxis[i]   = up.mV[i];
    }

    // Matrices -- LLMatrix4::mMatrix is a contiguous 4x4 row-major F32 block.
    const LLMatrix4& view = cam->getModelview();
    const LLMatrix4& proj = cam->getProjection();
    memcpy(f.mView, &view.mMatrix[0][0], 16 * sizeof(F32));
    memcpy(f.mProj, &proj.mMatrix[0][0], 16 * sizeof(F32));

    // G-buffer + scene color handles from the active render target pack.
    if (gPipeline.mRT)
    {
        LLRenderTarget& def = gPipeline.mRT->deferredScreen;
        LLRenderTarget& scr = gPipeline.mRT->screen;

        f.mWidth    = def.getWidth();
        f.mHeight   = def.getHeight();
        f.mTexDepth = def.getDepth();

        // deferredScreen attachment layout (see addDeferredAttachments):
        //   0 = albedo/diffuse, 1 = ORM (occ/rough/metal), 2 = normals, 3 = emissive(opt)
        const U32 n = def.getNumTextures();
        if (n > 0) f.mTexAlbedo   = def.getTexture(0);
        if (n > 1) f.mTexORM      = def.getTexture(1);
        if (n > 2) f.mTexNormals  = def.getTexture(2);
        if (n > 3) f.mTexEmissive = def.getTexture(3);

        if (scr.getNumTextures() > 0)
        {
            f.mTexColorHDR = scr.getTexture(0);
        }

        // Consider the frame usable only if the two effect-critical buffers exist.
        f.mValid = (f.mTexNormals != 0) && (f.mTexDepth != 0);
    }

#if LL_RESHADE_ADDON
    if (mEnabled && f.mValid)
    {
        // pushToReShade(f);  // implemented against real reshade.hpp once vendored
    }
#endif
}

// -----------------------------------------------------------------------------
// Add-on lifecycle. No-ops unless LL_RESHADE_ADDON is enabled.
// -----------------------------------------------------------------------------
void LLReShadeBridge::init()
{
#if LL_RESHADE_ADDON
    // Integration contract (ReShade 6.x add-on API, to be written against the
    // vendored reshade.hpp -- NOT from memory):
    //
    //   1. reshade::register_addon(hSelfModule)      // ReShade is injected into
    //                                                // THIS process, so we register
    //                                                // the viewer's own module.
    //   2. reshade::register_event<reshade::addon_event::init_effect_runtime>(...)
    //        -> cache the effect_runtime* so pushToReShade can talk to it.
    //   3. reshade::register_event<reshade::addon_event::reshade_begin_effects>(...)
    //        -> each frame: feed uniforms + texture bindings from mFrame.
    //
    // Uniforms (matrices, near/far, fov, camera frame):
    //   effect_runtime::enumerate_uniform_variables(nullptr, cb) and match a
    //   custom "source" annotation, then set_uniform_value_float(...).
    //
    // Textures (depth/normals/ORM/albedo/emissive/HDR color):
    //   wrap each raw GL name as a reshade::api::resource_view in ReShade's GL
    //   handle encoding, then effect_runtime::update_texture_bindings("SEMANTIC", view).
    //   NOTE: the GL-name -> resource_view encoding is the one detail that MUST be
    //   confirmed against the ReShade GL add-on examples; do not guess it.
    //
    // mEnabled = true;  // set once init_effect_runtime has fired
#endif
    mEnabled = false;
}

void LLReShadeBridge::shutdown()
{
#if LL_RESHADE_ADDON
    // reshade::unregister_addon(hSelfModule);
#endif
    mEnabled = false;
}
