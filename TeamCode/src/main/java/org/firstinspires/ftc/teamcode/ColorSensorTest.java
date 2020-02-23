package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import android.widget.RelativeLayout;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.api.Robot;

public class ColorSensorTest extends LinearOpMode {

    private int scaleFactor;
    private Robot bot;

    public void runOpMode() {
        this.bot = new Robot(hardwareMap, telemetry);
        this.scaleFactor = 1;

        bot.addColorSensor("color", scaleFactor);

        waitForStart();

        int layoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View layout = ((Activity) hardwareMap.appContext).findViewById(layoutId);

        while(!isStopRequested()){
            final int[] color = bot.getColorRGBA("color");

            layout.post(new Runnable() {
                public void run() {
                    layout.setBackgroundColor(Color.argb(color[3], color[0], color[1], color[2]));
                }
            });
        }

        layout.post(new Runnable() {
            public void run() {
                layout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}
