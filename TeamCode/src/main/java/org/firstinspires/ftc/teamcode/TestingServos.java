package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.api.Robot;

public class TestingServos extends OpMode {

    private Robot bot;
    private boolean bumperUp;

    public void init(){
        bumperUp = false;
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addServo("bumper", 180, 180, 90, 90);
    }

    public void loop(){
        boolean bumperRight = gamepad1.right_bumper;
        boolean bumperLeft = gamepad1.left_bumper;

        if(bumperLeft) {
            bot.incrementServo("bumper", 1, 0);
        }else if(bumperRight || bumperUp){
            bot.incrementServo("bumper", -1, 0);
        }

        telemetry.addData("Servo angle", bot.getServoPos("bumper"));
    }

}
