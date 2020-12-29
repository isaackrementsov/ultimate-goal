package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.api.Robot;

import java.util.Arrays;

@TeleOp
public class Drive extends OpMode {

    private Robot bot;

    private double power = 0.5;
    private double launcherSpeed = 0.8;

    private final String VUFORIA_KEY = "AY3aN3z/////AAABmUIe2Kd1wEt0nkr2MAal4OQiiEFWa3aLCHRnFBO1wd2HDT+GFXOTpcrhqEiZumOHpODdyVc55cYOiTSxpPrN+zfw7ZYB8X5z3gRLRIhPj4BJLD0/vPTKil7rDPSluUddISeCHL1HzPdIfiZiG/HQ89vhBdLfrWpngKLF4tH4FB4YWdKZu5J9EBtVTlXqR1OUXVTM3p9DepM9KukrVxMESF/ve+RYix7UXMO5qbljnc/LjQdplFO8oX4ztEe3aMXN14GadXggrfW+0m3nUmT8rXNTprc62LR1v0RbB4L+0QWfbgSDRyeMdBrvg8KIKLb1VFVrgUecbYBtHTTsLZALnU7oOOARnfGdtHC0aG3FAGxg";
    private final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private final String QUAD_LABEL = "Quad";
    private final String SINGLE_LABEL = "Single";

    // Using a custom state instead of saving entire gamepad1 (doing otherwise causes lag)
    private Robot.ButtonState lastButtons1 = new Robot.ButtonState();
    private Robot.DpadState lastDpads1 = new Robot.DpadState();
    private Robot.BumperState lastBumpers1 = new Robot.BumperState();

    private double offset = -55;
    private double[] armPositions = new double[]{-90 + offset, 0};
    private int currentPositionIndex = 0;

    public void init(){
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDrivetrain(new String[]{"mRF", "mLF", "mRB", "mLB"}, true);

        bot.addDcMotor("intake", true);
        bot.addDcMotor("intakeWheels", false);

        bot.addDcMotor("launcher", false);
        bot.runAtConstantVelocity("launcher");

        bot.addLimitedMotor("arm", "armLimit", "armLimit", 360, 3*288, true);
        bot.moveToStaticPosition("arm", 0, 0, true);

        bot.addServo("flipper");
        bot.rotateServo("flipper", 120, 0);

        bot.addServo("claw", 270, 180, 0);
        bot.rotateServo("claw", 0, 0);
    }

    public void start(){
        bot.resetLimitedMotor("arm", 0.2);
    }

    private void loopGamepad1(){
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y; // Reads negative from the controller
        double triggerRight = gamepad1.right_trigger;
        double triggerLeft = gamepad1.left_trigger;

        boolean bumperRight = gamepad1.right_bumper;
        boolean bumperLeft = gamepad1.left_bumper;
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadRight = gamepad1.dpad_right;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        boolean bumperRightHit = gamepad1.right_bumper && !lastBumpers1.right_bumper;
        boolean bumperLeftHit = gamepad1.left_bumper && !lastBumpers1.left_bumper;
        boolean dpadUpHit = dpadUp && !lastDpads1.dpad_up;
        boolean dpadDownHit = dpadDown && !lastDpads1.dpad_down;
        boolean dpadRightHit = dpadRight && !lastDpads1.dpad_right;
        boolean dpadLeftHit = dpadLeft && !lastDpads1.dpad_left;
        boolean xHit = x && !lastButtons1.x;
        boolean yHit = y && !lastButtons1.y;
        boolean aHit = a && !lastButtons1.a;

        // Use dpads to increment power
        double increment = 0.05;
        if(dpadUpHit){
            if(power < 1 - increment) power += increment;
        }else if(dpadDownHit){
            if(power > increment) power -= increment;
        }
        telemetry.addData("Drivetrain power", 100*power + "%");

        // Drive the robot with joysticks if they are moved
        if(Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1) {
            bot.drive(power, leftX, rightX, rightY);
        }else{
            // If the joysticks are not pressed, do not move the bot
            bot.stop();
        }

        // Reverse the drivetrain (for testing)
        if(gamepad1.left_stick_button){
            bot.reverseDrivetrain();
        }

        double intakePower = 0;
        if(Math.abs(triggerRight) > 0.05) {
            intakePower = triggerRight;
        }else if(Math.abs(triggerLeft) > 0.05){
            intakePower = -triggerLeft;
        }

        bot.moveDcMotor("intake", 2*intakePower);


        // Power the Wobble Goal Arm
        double maxSpeed = 0.7;
        if(bumperLeftHit){
            if(currentPositionIndex < armPositions.length - 1) currentPositionIndex++;
            else currentPositionIndex--;

            bot.moveDcMotor("arm", armPositions[currentPositionIndex] - bot.getMotorPosition("arm"), maxSpeed, true);
        }
        // (Optional) custom controller
        else{
            double error = bot.getTargetPosition("arm") - bot.getMotorPosition("arm");
            double threshold = 5;

            bot.moveDcMotor("arm", bot.getControlledSpeed(maxSpeed, threshold, error));
        }

        // Move the claw servo
        if(bumperRightHit){
            telemetry.addData("claw moving", bot.getServoPos("claw"));

            if(bot.getServoPos("claw") > 0){
                bot.rotateServo("claw", 0, 0);
            }else{
                bot.rotateServo("claw", 90, 0);
            }
        }

        // Move the launcher servo
        if(aHit){
            bot.rotateServo("flipper", 180, 250);
            bot.rotateServo("flipper", 120, 0);
        }

        // Turn intake wheels on/off
        double intakeWheelPower = bot.getMotorPower("intakeWheels");
        if(xHit){
            if(intakeWheelPower == 0){
                bot.moveDcMotor("intakeWheels", 1);
            }else{
                bot.moveDcMotor("intakeWheels", 0);
            }
        }else if(b && intakeWheelPower <= 0){
            bot.moveDcMotor("intakeWheels", -1);
        }else if(intakeWheelPower < 0){ //Reverse mode should only run when b is held
            bot.moveDcMotor("intakeWheels", 0);
        }

        // Turn launcher flywheel on/off
        if(yHit){
            if(bot.getMotorPower("launcher") == 0){
                bot.moveDcMotor("launcher", launcherSpeed);
            }else{
                bot.moveDcMotor("launcher", 0);
            }
        }
        // Change launcher speed
        double speedIncrement = 0.05;
        if(dpadRightHit){
            if(launcherSpeed < 1 - speedIncrement) launcherSpeed += speedIncrement;
        }else if(dpadLeftHit){
            if(launcherSpeed > speedIncrement) launcherSpeed -= speedIncrement;
        }
        if((dpadRightHit || dpadLeftHit) && bot.getMotorPower("launcher") > 0){
            bot.moveDcMotor("launcher", launcherSpeed);
        }
        telemetry.addData("Launcher speed", 100*launcherSpeed + "%");

        // Save button states
        lastButtons1.update(a, b, x, y);
        lastDpads1.update(dpadUp, dpadDown, dpadRight, dpadLeft);
        lastBumpers1.update(bumperRight, bumperLeft);
    }

    @Override
    public void loop(){
        loopGamepad1();
    }
}
