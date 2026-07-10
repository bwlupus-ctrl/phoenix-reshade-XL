/**
 * @file llfloaterflycamrecorder.h
 * @brief Transport UI for the flycam path recorder (LLFlycamRecorder).
 *
 * $LicenseInfo:firstyear=2026&license=viewerlgpl$
 * Phoenix / Firestorm Viewer Source Code
 * $/LicenseInfo$
 */

#ifndef LL_LLFLOATERFLYCAMRECORDER_H
#define LL_LLFLOATERFLYCAMRECORDER_H

#include "llfloater.h"

class LLButton;
class LLSliderCtrl;
class LLTextBox;

class LLFloaterFlycamRecorder : public LLFloater
{
public:
    LLFloaterFlycamRecorder(const LLSD& key);

    bool postBuild() override;
    void draw() override;

private:
    void onRecord();
    void onPlayPause();
    void onStop();
    void onClear();
    void onSave();
    void onLoad();
    void onScrub();

    LLButton*     mRecordBtn = nullptr;
    LLButton*     mPlayBtn = nullptr;
    LLSliderCtrl* mScrub = nullptr;
    LLTextBox*    mTimeText = nullptr;
    LLTextBox*    mStatusText = nullptr;
};

#endif // LL_LLFLOATERFLYCAMRECORDER_H
