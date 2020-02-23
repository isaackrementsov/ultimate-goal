package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.api.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest extends LinearOpMode {

    private Robot bot;


    public void runOpMode() {
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addServo("sideArm", 180, 180, 0);

        waitForStart();

        while(!isStopRequested()){
            bot.rotateServo("sideArm", 20, 2000);
            bot.rotateServo("sideArm", 160, 2000);
        }
    }

    /*
    private void timeout(int millis){
        try {
            Thread.sleep(5000);
        }catch(InterruptedException e){}
    }
    */

}