// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.music;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Play audio via the speakers on CTRE Falcon 500 motors. */
public class BreakerFalconOrchestra extends SubsystemBase {

    private Orchestra orchestra;
    private String[] currentPlaylist;
    private String loopedSong;
    private int nextPlaylistSong = 0;
    private boolean runPlaylist = false;
    private boolean runLooped = false;

    public BreakerFalconOrchestra() {
        orchestra = new Orchestra();
    }

    public BreakerFalconOrchestra(WPI_TalonFX... motors) {
        Collection<TalonFX> instruments = new ArrayList<>();
        for (TalonFX motor : motors) {
            instruments.add(motor);
        }
        orchestra = new Orchestra(instruments);
    }

    public void addOrchestraMotors(WPI_TalonFX... motors) {
        for (WPI_TalonFX motor : motors) {
            orchestra.addInstrument(motor);
        }
    }

    /** Starts going through playlist. Must be Chirp files. */
    public void startPlaylist(String[] playlistSongFilepaths) {
        currentPlaylist = playlistSongFilepaths;
        runPlaylist = true;
    }

    /** Loops the given song. Must be a Chirp file. */
    public void startLoopedSong(String loopSongFilepath) {
        loopedSong = loopSongFilepath;
        runLooped = true;
    }

    /** Stops playing the playlist. */
    public void stopPlaylist() {
        runPlaylist = false;
        nextPlaylistSong = 0;
    }

    /** Stops playing the looped song. */
    public void stopLoopedSong() {
        runLooped = false;
    }

    /** Stops playing the current song and resets playback position to beginning. */
    public void stopMusic() {
        orchestra.stop();
    }

    /** Pauses the current song, allowing for the song to be resumed later. */
    public void pauseMusic() {
        orchestra.pause();
    }

    /** Plays the current song. Resumes if previously paused. */
    public void playMusic() {
        orchestra.play();
    }

    /** Loads given Chirp file. Stops current playlist. */
    public void loadMusic(String musicFilepath) {
        stopPlaylist();
        orchestra.loadMusic(musicFilepath);
    }

    /** If music playback is paused. */
    public boolean isPaused() {
        return !(orchestra.getCurrentTime() == 0) && !orchestra.isPlaying();
    }

    /** If music playback is stopped. */
    public boolean isStopped() {
        return (orchestra.getCurrentTime() == 0) && !orchestra.isPlaying();
    }

    /** Current song timestamp in milliseconds. */
    public int getMusicTimestampMS() {
        return orchestra.getCurrentTime();
    }

    /** Loop for playing through a playlist. */
    private void runPlaylistLoop() {
        if (runPlaylist && isStopped()) {
            try {
                orchestra.loadMusic(currentPlaylist[nextPlaylistSong]);
                orchestra.play();
                nextPlaylistSong++;
            } catch (Exception e) { // Stops music playback if an exception occurs.
                e.printStackTrace();
                nextPlaylistSong = 0;
                orchestra.stop();
            }
        }
    }

    /** Looped song plays. */
    private void runLoopedSong() {
        if (runLooped && isStopped()) {
            orchestra.loadMusic(loopedSong);
            orchestra.play();
        }
    }

    @Override
    public void periodic() {
        runPlaylistLoop();
        runLoopedSong();
    }
}
