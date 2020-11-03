package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.api.Robot;

@TeleOp
public class Drive extends OpMode {

    private Robot bot;
    private double power;

    public void init(){
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDrivetrain(new String[]{"mRF", "mLF", "mRB", "mLB"}, true);
        bot.addDcMotor("intake", true);

        power = 0.5;
    }

    public void loop(){
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y; // Reads negative from the controller
        double triggerRight = gamepad1.right_trigger;
        double triggerLeft = gamepad1.left_trigger;

        if (gamepad1.dpad_up) power = 0.9;
        if (gamepad1.dpad_right) power = 0.5;
        if (gamepad1.dpad_left) power = 0.3;
        if (gamepad1.dpad_down) power = 0.2;

        telemetry.addData("RightY", rightY);
        telemetry.addData("RightX", rightX);
        telemetry.addData("LeftX", leftX);

        // Drive the robot with joysticks if they are moved
        if (Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1) {
            bot.drive(power, leftX, rightX, rightY);
        } else {
            // If the joysticks are not pressed, do not move the bot
            bot.stop();
        }

        double intakePower = 0;
        if(Math.abs(triggerRight) > 0.05) {
            intakePower = triggerRight;
        }else if(Math.abs(triggerLeft) > 0.05){
            intakePower = -triggerLeft;
        }else{
            intakePower = 0;
        }

        bot.moveDcMotor("intake", intakePower);
    }

}
