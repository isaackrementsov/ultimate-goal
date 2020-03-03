package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.api.Robot;

@TeleOp
public class TeleOpAPIExample extends OpMode {

    private Robot bot;
    private double power = 0.2;


    public void init(){
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDrivetrain(new String[]{"mRF", "mLF", "mRB", "mLB"}, true);
    }

    public void loop() {
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double rightY = gamepad1.right_stick_y;

        leftX = Range.clip(leftX, -1, 1);
        rightX = Range.clip(rightX, -1, 1);
        rightY = Range.clip(rightY, -1, 1);

        if(gamepad1.dpad_up) power = 2;
        if(gamepad1.dpad_right) power = 1;
        if(gamepad1.dpad_left) power = 0.2;
        if(gamepad1.dpad_down) power = 0.1;


        if(Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1){
            bot.drive(-power, leftX, rightX, rightY);
        }else{
            bot.stop();
        }
    }

}
