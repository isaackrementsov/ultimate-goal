package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.api.Robot;

@TeleOp
public class Drive extends OpMode {

    private Robot bot;
    private double power;

    private final String VUFORIA_KEY = "AY3aN3z/////AAABmUIe2Kd1wEt0nkr2MAal4OQiiEFWa3aLCHRnFBO1wd2HDT+GFXOTpcrhqEiZumOHpODdyVc55cYOiTSxpPrN+zfw7ZYB8X5z3gRLRIhPj4BJLD0/vPTKil7rDPSluUddISeCHL1HzPdIfiZiG/HQ89vhBdLfrWpngKLF4tH4FB4YWdKZu5J9EBtVTlXqR1OUXVTM3p9DepM9KukrVxMESF/ve+RYix7UXMO5qbljnc/LjQdplFO8oX4ztEe3aMXN14GadXggrfW+0m3nUmT8rXNTprc62LR1v0RbB4L+0QWfbgSDRyeMdBrvg8KIKLb1VFVrgUecbYBtHTTsLZALnU7oOOARnfGdtHC0aG3FAGxg";
    private final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private final String QUAD_LABEL = "Quad";
    private final String SINGLE_LABEL = "Single";

    // Using a custom state instead of saving entire gamepad1 (doing otherwise causes lag)
    private Robot.ButtonState lastButtons1 = new Robot.ButtonState();
    private Robot.DpadState lastDpads1 = new Robot.DpadState();
    private Robot.BumperState lastBumpers1 = new Robot.BumperState();

    private int[] armPositions = new int[]{-90, 0, 95};
    private int currentPositionIndex = 1;

    public void init(){
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDrivetrain(new String[]{"mRF", "mLF", "mRB", "mLB"}, true);

        bot.addDcMotor("intake", true);
        bot.addDcMotor("intakeWheels", false);

        bot.addDcMotor("launcher", false);
        bot.runAtConstantVelocity("launcher");

        bot.addDcMotor("arm", 360, 288, true, true);
        bot.runAtConstantVelocity("arm");
        bot.moveDcMotor("arm", 0, 0.5, true);

        bot.addServo("flipper");
        bot.rotateServo("flipper", 180, 0);

        power = 0.5;
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
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        boolean bumperRightHit = gamepad1.right_bumper && !lastBumpers1.right_bumper;
        boolean bumperLeftHit = gamepad1.left_bumper && !lastBumpers1.left_bumper;
        boolean dpadUpHit = dpadUp && !lastDpads1.dpad_up;
        boolean dpadDownHit = dpadDown && !lastDpads1.dpad_down;
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
        telemetry.addData("Motor power:", power);

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
            intakePower = -triggerRight;
        }else if(Math.abs(triggerLeft) > 0.05){
            intakePower = triggerLeft;
        }

        bot.moveDcMotor("intake", 2*intakePower);


        // Power the Wobble Goal Arm
        double maxSpeed = 0.9;
        double armSpeed = 0;
        if(bumperLeftHit || bumperRightHit){
            if(bumperLeftHit){
                if(currentPositionIndex < armPositions.length - 1) currentPositionIndex++;
            }else if(bumperRightHit){
                if(currentPositionIndex > 0) currentPositionIndex--;
            }

            bot.moveToStaticPosition("arm", armPositions[currentPositionIndex], maxSpeed, true);
        }else{
            double error = bot.getTargetPosition("arm") - bot.getMotorPosition("arm");

            telemetry.addData("Error:", error);
            telemetry.addData("Speed", Math.signum(error) * maxSpeed);
            bot.moveDcMotor("arm", Math.signum(error) * maxSpeed);
        }

        // Move the arm between 0 and 180 degrees at 90 degree increments

        if(aHit){
            bot.rotateServo("flipper", 120, 250);
            bot.rotateServo("flipper", 180, 0);
        }

        // Turn intake wheels on/off
        if(xHit){
            if(bot.getMotorPower("intakeWheels") == 0){
                bot.moveDcMotor("intakeWheels", 1);
            }else{
                bot.moveDcMotor("intakeWheels", 0);
            }
        }else if(b){
            bot.moveDcMotor("intakeWheels", -1);
        }else if(bot.getMotorPower("intakeWheels") < 0){ //Reverse mode should only run when b is held
            bot.moveDcMotor("intakeWheels", 0);
        }

        // Turn launcher flywheel on/off
        if(yHit){
            if(bot.getMotorPower("launcher") == 0){
                bot.moveDcMotor("launcher", 0.8);
            }else{
                bot.moveDcMotor("launcher", 0);
            }
        }

        // Save button states
        lastButtons1.update(a, b, x, y);
        lastDpads1.update(dpadUp, dpadDown);
        lastBumpers1.update(bumperRight, bumperLeft);
    }

    @Override
    public void loop(){
        loopGamepad1();
    }

    private boolean xyabOn(Gamepad gamepad){ return gamepad.a || gamepad.b || gamepad.x || gamepad.y; }
}
