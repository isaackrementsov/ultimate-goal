package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.api.Robot;

@TeleOp
public class NewRobotTeleOp extends OpMode {

    private Robot bot;
    private double power = 0.2;
    private boolean bumperUp;

    public void init() {
        bumperUp = false;

        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDrivetrain(new String[]{"mRF", "mLF", "mRB", "mLB"}, true);

        bot.addDcMotor("intake1", true);
        bot.addDcMotor("intake2", true);

        bot.addLimitedMotor("inOut", "limitOut", "limitIn", true);
        // Limit switch positions have to be reversed because mechanical hooked up the motor backwards
        //bot.addLimitedMotor("upDown", new String[]{"limitUp1", "limitUp2"}, new String[]{"limitDown"}, true);
        bot.addDcMotor("upDown", true); // 288 encoder ticks

        bot.addServo("bumper", 180, 115, 60);
        bot.addServo("gripperTurn", 180, 180, 0, 5);
        bot.addServo("capstone", 180, 180, 0);
        bot.addServo("gripper", 180, 180, 0);
        bot.addServo("droopy", 180, 180, 30);

        //bot.resetServo("gripperTurn", 0);
        //bot.resetLimitedMotor("inOut");
    }

    public void loop() {
        processController1();
        processController2();
    }

    private void processController1() {
        // Driver controls
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double rightY = gamepad1.right_stick_y;

        // Keep joysticks between -1 and 1
        leftX = Range.clip(leftX, -1, 1);
        rightX = Range.clip(rightX, -1, 1);
        rightY = Range.clip(rightY, -1, 1);

        // Set different power modes
        if (gamepad1.dpad_up) power = 0.9;
        if (gamepad1.dpad_right) power = 0.5;
        if (gamepad1.dpad_left) power = 0.3;
        if (gamepad1.dpad_down) power = 0.2;

        // Drive the robot with joysticks if they are moved
        if (Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1) {
            bot.drive(power, leftX, rightX, rightY);
        } else {
            // If the joysticks are not pressed, do not move the bot
            bot.stop();
        }

        // Spin the two intake motors
        double triggerRight = gamepad1.right_trigger;
        double triggerLeft = gamepad1.left_trigger;

        // Keep triggers between 0 and 1
        triggerRight = Range.clip(triggerRight, 0, 1);
        triggerLeft = Range.clip(triggerLeft, 0, 1);

        double intakePower = 0;

        // Set the intake power based on triggers if they are pressed
        if (triggerRight > .1) intakePower = 4;
        else if (triggerLeft > .1) intakePower = -4;

        bot.moveDcMotor("intake1", intakePower);
        bot.moveDcMotor("intake2", -intakePower);

        // Run the foundation grabber
        boolean bumperRight = gamepad1.right_bumper;
        boolean bumperLeft = gamepad1.left_bumper;

        double pos = 60;

        if (bumperLeft) {
            bumperUp = false;
            pos = 60;
        } else if (bumperRight || bumperUp) {
            bumperUp = true;
            pos = 115;
        }

        bot.rotateServo("bumper", pos, 0);
    }

    private void processController2() {

        // Arm motor in-out, controlled by right joystick Y
        double leftY = gamepad2.left_stick_y;
        double rightY = gamepad2.right_stick_y;
        double triggerRight = gamepad2.right_trigger;
        double triggerLeft = gamepad2.left_trigger;

        //bot.resetLimitedMotor("inOut");

        rightY = Range.clip(rightY, -1, 1);
        leftY = Range.clip(leftY, -1, 1);
        telemetry.addData("rightY", rightY);
        // Make sure joy is being moved
        if (Math.abs(rightY) > .1) {
            bot.moveLimitedMotor("inOut", rightY);
        } else {
            bot.moveDcMotor("inOut", 0);
        }

        if(Math.abs(leftY) > .01) {
            bot.rotateServo("droopy", 125, 0);
        } else {
            bot.rotateServo("droopy", 32.5, 0);
        }

        // Eva controls

        /*
        if (gamepad2.a) {
            // capstone servo move to 180 degrees
            bot.rotateServo("capstone", 180.0, 0);
        }
        if (gamepad2.y) {
            bot.rotateServo("gripper", 180.0, 0);
            bot.moveDcMotor("upDown", 30, 0.2);
        }
        if (gamepad2.x) {
            bot.rotateServo("gripper", 180.0, 0);
            bot.moveLimitedMotor("inOut", 30, 0.2);
        } else {
            bot.rotateServo("capstone", 0, 0);
        }
        */

        if(gamepad2.a && gamepad2.dpad_down) {
            bot.rotateServo("capstone", 30, 0);
        }

        // Rotate gripper servo, controlled by left joystick X (precise) and dpad (general)
        boolean dpadLeft = gamepad2.dpad_left;
        boolean dpadRight = gamepad2.dpad_right;

        telemetry.addData("Dpad right", dpadRight);
        telemetry.addData("dpad left", dpadLeft);

        // Use dpads to move rigidly between 0 and 180
        if (gamepad2.dpad_right) {
            bot.rotateServo("gripperTurn", 3, 0);
        } else if (gamepad2.dpad_left) {
            bot.rotateServo("gripperTurn", 180, 0);
        } else if (gamepad2.dpad_up) {
            bot.rotateServo("gripperTurn", 90, 0);
        }

        // Move the arm up and down with triggers

        // If triggers are pressed, move up/down with limit switches
        /*
        if (Math.abs(leftY) > .1) {
            bot.moveLimitedMotorArray("upDown", 0.9 * leftY, Robot.LimitBehavior.AND);
        } else {
            bot.moveDcMotor("upDown", 0);
        }
        */


        // Close/Open grabber motor using bumpers
        boolean bumperR = gamepad2.right_bumper;
        boolean bumperL = gamepad2.left_bumper;


        if (bumperL) {
            bot.rotateServo("gripper", 0, 0);
        } else if (bumperR) {
            bot.rotateServo("gripper", 180, 0);
        }


        if(triggerRight > 0.1) {
            bot.moveDcMotor("upDown", .6);
        }
        else if (triggerLeft > 0.1) {
            bot.moveDcMotor("upDown", -.6);
        }
        else {
            bot.moveDcMotor("upDown", 0);
        }


    }
}

