package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

/**
 * Created by Doug on 2/26/2018.
 */

public class AudioPlayer extends Thread {

    private static String source = null;
    
    public static void play(String file)
    {
        AudioPlayer ap = new AudioPlayer(file);
        ap.start();
    }

    public AudioPlayer(String name)
    {
        source = name;
    }

    @Override
    public void run()
    {
        MediaPlayer mp = new MediaPlayer();
        try {
            mp.setDataSource(source);
            mp.start();
        } catch (Exception e) {
            ;
        }
    }
}
