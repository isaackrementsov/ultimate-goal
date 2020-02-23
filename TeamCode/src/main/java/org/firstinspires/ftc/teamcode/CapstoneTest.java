package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.Robot;

@Autonomous
public class CapstoneTest extends LinearOpMode {

    private Robot bot;

    public void runOpMode(){
        this.bot = new Robot(hardwareMap, telemetry);
        bot.addServo("capstone", 180, 180, 0);

        waitForStart();

        int t = 1000;

        while(!isStopRequested()){
            bot.rotateServo("capstone", 180, t);
            bot.rotateServo("capstone", 120, t);
            bot.rotateServo("capstone", 60, t);
            bot.rotateServo("capstone", 30, t);
            bot.rotateServo("capstone", 0, t);
        }
    }

}
