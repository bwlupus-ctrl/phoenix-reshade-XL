/**
 * @file llfloaterflycamrecorder.cpp
 * @brief Transport UI for the flycam path recorder (LLFlycamRecorder).
 *
 * $LicenseInfo:firstyear=2026&license=viewerlgpl$
 * Phoenix / Firestorm Viewer Source Code
 * $/LicenseInfo$
 */

#include "llviewerprecompiledheaders.h"

#include "llfloaterflycamrecorder.h"

#include "llbutton.h"
#include "llflycamrecorder.h"
#include "llsliderctrl.h"
#include "lltextbox.h"
#include "llviewermenufile.h"   // LLFilePickerReplyThread

namespace
{
void pickerSave(const std::vector<std::string>& filenames,
                LLFilePicker::ELoadFilter, LLFilePicker::ESaveFilter)
{
    if (!filenames.empty())
    {
        LLFlycamRecorder::instance().saveToFile(filenames[0]);
    }
}

void pickerLoad(const std::vector<std::string>& filenames,
                LLFilePicker::ELoadFilter, LLFilePicker::ESaveFilter)
{
    if (!filenames.empty())
    {
        LLFlycamRecorder::instance().loadFromFile(filenames[0]);
    }
}
} // anonymous namespace

LLFloaterFlycamRecorder::LLFloaterFlycamRecorder(const LLSD& key)
:   LLFloater(key)
{
}

bool LLFloaterFlycamRecorder::postBuild()
{
    mRecordBtn = getChild<LLButton>("btn_record");
    mPlayBtn = getChild<LLButton>("btn_play");
    mScrub = getChild<LLSliderCtrl>("scrub_slider");
    mTimeText = getChild<LLTextBox>("time_lbl");
    mStatusText = getChild<LLTextBox>("status_lbl");

    childSetAction("btn_record", boost::bind(&LLFloaterFlycamRecorder::onRecord, this));
    childSetAction("btn_play", boost::bind(&LLFloaterFlycamRecorder::onPlayPause, this));
    childSetAction("btn_stop", boost::bind(&LLFloaterFlycamRecorder::onStop, this));
    childSetAction("btn_clear", boost::bind(&LLFloaterFlycamRecorder::onClear, this));
    childSetAction("btn_save", boost::bind(&LLFloaterFlycamRecorder::onSave, this));
    childSetAction("btn_load", boost::bind(&LLFloaterFlycamRecorder::onLoad, this));
    mScrub->setCommitCallback(boost::bind(&LLFloaterFlycamRecorder::onScrub, this));

    return true;
}

void LLFloaterFlycamRecorder::draw()
{
    LLFlycamRecorder& rec = LLFlycamRecorder::instance();
    const LLFlycamRecorder::EState state = rec.getState();
    const F32 duration = rec.getDuration();
    const bool recording = (state == LLFlycamRecorder::STATE_RECORDING);

    mRecordBtn->setLabel(recording ? LLStringExplicit("Stop Rec")
                                   : LLStringExplicit("Record"));
    mPlayBtn->setLabel(state == LLFlycamRecorder::STATE_PLAYING
                           ? LLStringExplicit("Pause")
                           : LLStringExplicit("Play"));
    mPlayBtn->setEnabled(rec.getNumKeyframes() > 0 && !recording);
    mScrub->setEnabled(rec.getNumKeyframes() > 0 && !recording);

    mScrub->setMaxValue(llmax(duration, 0.01f));
    mScrub->setValue(rec.getPlayhead());

    const F32 shown = recording ? duration : rec.getPlayhead();
    mTimeText->setText(llformat("%.1f / %.1f s   %d keys",
                                shown, duration, rec.getNumKeyframes()));
    mStatusText->setText(rec.getStatus());

    LLFloater::draw();
}

void LLFloaterFlycamRecorder::onRecord()
{
    LLFlycamRecorder& rec = LLFlycamRecorder::instance();
    if (rec.getState() == LLFlycamRecorder::STATE_RECORDING)
    {
        rec.stopRecording();
    }
    else
    {
        rec.startRecording();
    }
}

void LLFloaterFlycamRecorder::onPlayPause()
{
    LLFlycamRecorder::instance().togglePlayback();
}

void LLFloaterFlycamRecorder::onStop()
{
    LLFlycamRecorder& rec = LLFlycamRecorder::instance();
    if (rec.getState() == LLFlycamRecorder::STATE_RECORDING)
    {
        rec.stopRecording();
    }
    else
    {
        rec.stopPlayback();
    }
}

void LLFloaterFlycamRecorder::onClear()
{
    LLFlycamRecorder::instance().clear();
}

void LLFloaterFlycamRecorder::onSave()
{
    // callbacks go through the recorder singleton, so an early floater
    // close while the picker is up can't dangle
    LLFilePickerReplyThread::startPicker(&pickerSave, LLFilePicker::FFSAVE_XML,
                                         "flycam_take.xml");
}

void LLFloaterFlycamRecorder::onLoad()
{
    LLFilePickerReplyThread::startPicker(&pickerLoad, LLFilePicker::FFLOAD_XML, false);
}

void LLFloaterFlycamRecorder::onScrub()
{
    // commit only fires on user interaction (draw()'s setValue doesn't),
    // so this is always a deliberate scrub; from idle it previews the pose
    LLFlycamRecorder::instance().seek(mScrub->getValueF32());
}
