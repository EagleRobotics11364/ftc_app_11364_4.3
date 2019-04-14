package org.firstinspires.ftc.teamcode.library.functions

import android.media.MediaPlayer
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.R

class MusicPlayer (val hardwareMap: HardwareMap) {
    private val mediaPlayer: MediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.unity)

    init {
        mediaPlayer.isLooping = true
        mediaPlayer.seekTo(0)
    }

    fun play() {
        if (!mediaPlayer.isPlaying) mediaPlayer.start()
    }
    fun pause() {
        mediaPlayer.pause()
    }
    fun stop() {
        mediaPlayer.stop()
        mediaPlayer.release()
    }
    fun isPlaying() = mediaPlayer.isPlaying
}