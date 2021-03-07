package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.api.ControlledDrivetrain;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.firstinspires.ftc.teamcode.api.Robot;

@TeleOp
public class DriveV2 extends OpMode {

    private Robot bot;

    // Odometry parameters
    private int ticksPerRev = 8192;
    private double circumference = 15.71;
    private double width = 40.8;
    private double backDistancePerRadian = -41.577/(2*Math.PI);

    // Where the robot starts (in cm from the right wall)
    private double x0 = 112.395;
    private double y0 = 0;
    private double phi0 = 0;

    private ControlledDrivetrain drivetrain;

    private double launcherSpeed = 0.68;

    // Using a custom state instead of saving entire gamepad1 (doing otherwise causes lag)
    private Robot.ButtonState lastButtons1 = new Robot.ButtonState();
    private Robot.DpadState lastDpads1 = new Robot.DpadState();
    private Robot.BumperState lastBumpers1 = new Robot.BumperState();

    private double lastTimeHit = System.currentTimeMillis();
    private boolean flipperClosed = false;

    private double offset = -60;
    private double[] armPositions = new double[]{27 + offset, -45 + offset, -90 + offset};
    private int currentPositionIndex = 0;

    private double zeroPositionX = 195.95;
    private double zeroPositionY = -159.4;

    private double[] shootingPositionsX = new double[]{174.6, 160.2, 135.9, 100.2};
    private double[] shootingPositionsY = new double[]{-146,-150.3,-153.47,-151.1};
    private double[] shootingSpeeds = new double[]{0.63, 0.63, 0.63, 0.67};
    private int shootingIndex = shootingPositionsX.length - 1;

    public void init(){
        // Get all of the drivetrain motors
        DcMotorX mRF = new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF")),
                mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB")),
                mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));
        // Get the odometry wheels
        DcMotorX wheelR = new DcMotorX(hardwareMap.dcMotor.get("mRB"), ticksPerRev, circumference),
                wheelL = new DcMotorX(hardwareMap.dcMotor.get("mLF"), ticksPerRev, circumference),
                wheelB = new DcMotorX(hardwareMap.dcMotor.get("mRF"), ticksPerRev, circumference);

        // Create an odometry instance for the drivetrain
        Odometry positionTracker = new Odometry(wheelR, wheelL, wheelB, 50, backDistancePerRadian, width, x0, y0, phi0);

        // Instantiate the PID-controlled drivetrain
        drivetrain = new ControlledDrivetrain(mRF, mLF, mRB, mLB, positionTracker);
        // Reverse the drivetrain (between left/right wheels are on backward)
        drivetrain.reverse();
        // Adding logging to drivetrain (only needed for development)
        drivetrain.telemetry = telemetry;
        // Start with the drivetrain off
        drivetrain.setActive(false);
        // Run it on a separate thread
        Thread drivetrainThread = new Thread(drivetrain);
        drivetrainThread.start();

        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDcMotor("intake", true);
        bot.addDcMotor("intakeWheels", false);

        bot.addDcMotor("launcher", false);
        bot.runAtConstantVelocity("launcher");

        bot.addLimitedMotor("arm", "armLimit", "armLimit", 360, 3*288, true);
        bot.moveToStaticPosition("arm", 0, 0, true);

        bot.addServo("flipper");
        bot.rotateServo("flipper", 50, 0);

        bot.addServo("indicator");
        bot.rotateServo("indicator", 70 - 20*shootingIndex, 0);

        bot.addServo("claw", 270, 180, 0);
        bot.rotateServo("claw", 100, 0);
    }

    public void start(){
        bot.resetLimitedMotor("arm", 0.2);
        bot.moveDcMotor("arm", armPositions[currentPositionIndex] - bot.getMotorPosition("arm"), 0.7, true);
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

        if(dpadUpHit){
            int max = shootingPositionsX.length - 1;
            shootingIndex = shootingIndex == max ? 0 : shootingIndex + 1;

            bot.rotateServo("indicator", 70 - 20*shootingIndex, 0);
            launcherSpeed = shootingSpeeds[shootingIndex];
        }else if(dpadDownHit){
            drivetrain.setPosition(shootingPositionsX[shootingIndex], shootingPositionsY[shootingIndex], getClosestAngleToZero());
            drivetrain.setActive(true);
        }

        // Drive the robot with joysticks if they are moved
        if(Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1) {
            if(drivetrain.getActive()) drivetrain.setActive(false);
            drivetrain.driveWithGamepad(1, rateCurve(rightY, 1.7),0.5*leftX, rateCurve(rightX,1.7));      //curved stick rates
        }else{
            // If the joysticks are not pressed, do not move the bot
            if(!drivetrain.getActive()) drivetrain.stop();
        }

        // Reverse the drivetrain (for testing)
        if(gamepad1.left_stick_button){
            bot.reverseDrivetrain();
        }

        if(gamepad1.right_stick_button){
            // Reset odometry to prevent error buildup
            drivetrain.positionTracker.reset(zeroPositionX, zeroPositionY, 0);
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
        telemetry.addData("current index", currentPositionIndex);
        if(bumperLeftHit){
            if(currentPositionIndex < armPositions.length - 1) currentPositionIndex++;
            else currentPositionIndex = 0;

            bot.moveDcMotor("arm", armPositions[currentPositionIndex] - bot.getMotorPosition("arm"), maxSpeed, true);
        }
        // (Optional) custom controller
        else{
            double error = bot.getTargetPosition("arm") - bot.getMotorPosition("arm");
            double threshold = 2;

            bot.moveDcMotor("arm", bot.getControlledSpeed(maxSpeed, threshold, error, true));
        }

        // Move the claw servo
        if(bumperRightHit){
            if(bot.getServoAngle("claw") > 0){
                bot.rotateServo("claw", 0, 0);
            }else{
                bot.rotateServo("claw", 100, 0);
            }
        }

        // Move the launcher servo
        double timeElapsed = System.currentTimeMillis() - lastTimeHit;

        if((aHit || (a && timeElapsed > 450)) && bot.getMotorPower("launcher") > 0){
            lastTimeHit = System.currentTimeMillis();
            flipperClosed = false;
            bot.rotateServo("flipper", 25, 0);
        }else if(timeElapsed > 200 && !flipperClosed){
            flipperClosed = true;
            bot.rotateServo("flipper", 50, 0);
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

        if(dpadRightHit){
            launcherSpeed = 0.68;
        }else if(dpadLeftHit){
            launcherSpeed = 0.6;
        }

        if((dpadRightHit || dpadLeftHit) && bot.getMotorPower("launcher") > 0){
            bot.moveDcMotor("launcher", launcherSpeed);
        }
        telemetry.addData("Launcher speed", 100*launcherSpeed + "%");

        // Save button states
        lastButtons1.update(a, b, x, y);
        lastDpads1.update(dpadUp, dpadDown, dpadRight, dpadLeft);
        lastBumpers1.update(bumperRight, bumperLeft);

        telemetry.addData("x", drivetrain.positionTracker.x);
        telemetry.addData("y", drivetrain.positionTracker.y);
        telemetry.addData("Heading", drivetrain.positionTracker.phi);
        telemetry.addData("Target heading", getClosestAngleToZero());
        telemetry.addData("Is the drivetrain active?", drivetrain.getActive());
    }

    private double rateCurve(double input, double rate){
        return Math.pow(Math.abs(input),rate)*((input>0)?1:-1);
    }

    private double getClosestAngleToZero(){
        return Math.round(drivetrain.positionTracker.phi/2*Math.PI)*2*Math.PI;
    }

    public void stop(){
        drivetrain.setActive(false);
        drivetrain.stop();
        drivetrain.stopController();
    }

    @Override
    public void loop(){
        loopGamepad1();
    }
}
