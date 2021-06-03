package org.firstinspires.ftc.teamcode.api.examples.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.api.ControlledDrivetrain;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.LimitedMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.firstinspires.ftc.teamcode.api.ServoX;
import org.firstinspires.ftc.teamcode.api.State;

@TeleOp
public class Drive extends OpMode {

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
    private State.Buttons lastButtons1 = new State.Buttons();
    private State.Dpad lastDpads1 = new State.Dpad();
    private State.Bumpers lastBumpers1 = new State.Bumpers();

    private double lastTimeHit = System.currentTimeMillis();
    private boolean flipperClosed = false;

    private double offset = -60;
    private double[] armPositions = new double[]{27 + offset, -45 + offset, -90 + offset};
    private int currentPositionIndex = 0;

    private boolean moveForward = false;

    private double zeroPositionX = 195.95;
    private double zeroPositionY = -159.4;

    private double[] shootingPositionsX = new double[]{174.6, 160.2, 135.9, 100.2};
    private double[] shootingPositionsY = new double[]{-146,-150.3,-153.47,-151.1};
    private double[] shootingSpeeds = new double[]{0.63, 0.63, 0.63, 0.67};
    private int shootingIndex = shootingPositionsX.length - 1;

    // DcMotors
    private DcMotorX
        mRF,
        mLF,
        mRB,
        mLB,
        wheelR,
        wheelL,
        wheelB,
        intake,
        intakeWheels,
        launcher;

    // Limited motors
    private LimitedMotorX arm;

    // Servos
    private ServoX
        flipper,
        claw,
        indicator;

    public void init(){
        // Drivetrain motors
        mRF = new DcMotorX(hardwareMap.dcMotor.get("mRF"));
        mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF"));
        mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB"));
        mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));

        // Odometry wheels
        wheelR = new DcMotorX(hardwareMap.dcMotor.get("mRB"), ticksPerRev, circumference);
        wheelL = new DcMotorX(hardwareMap.dcMotor.get("mLF"), ticksPerRev, circumference);
        wheelB = new DcMotorX(hardwareMap.dcMotor.get("mRF"), ticksPerRev, circumference);

        // Intake mechanism
        intake = new DcMotorX(hardwareMap.dcMotor.get("intake"));
        intakeWheels = new DcMotorX(hardwareMap.dcMotor.get("intakeWheels"));
        intake.setBrake(true);

        // Constant-velocity ring launcher
        launcher = new DcMotorX(hardwareMap.dcMotor.get("launcher"));
        launcher.controlVelocity();
        // Flipper/shooter servo
        flipper = new ServoX(hardwareMap.servo.get("flipper"));
        flipper.setAngle(50);

        // Wobble arm
        arm = new LimitedMotorX(hardwareMap.dcMotor.get("arm"), 3*288, 360);
        arm.setLowerLimit(hardwareMap.touchSensor.get("armLimit"));
        arm.setBrake(true);
        // Wobble claw
        claw = new ServoX(hardwareMap.servo.get("claw"), 270, 180);
        claw.setAngle(100);

        // Drive-assist indicator
        indicator = new ServoX(hardwareMap.servo.get("indicator"));
        indicator.setAngle(70 - 20*shootingIndex);

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
    }

    public void start(){
        arm.reset();
        arm.resetEncoder();
        arm.controlPosition();
        arm.setPosition(armPositions[currentPositionIndex], 0.7);
    }

    public void loopGamepad1(){
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

            indicator.setAngle(70 - 20*shootingIndex);
            launcherSpeed = shootingSpeeds[shootingIndex];
        }else if(dpadDownHit){
            drivetrain.setPosition(
                    shootingPositionsX[shootingIndex],
                    shootingPositionsY[shootingIndex],
                    getClosestAngleToZero(drivetrain.positionTracker.phi)
            );

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
            drivetrain.reverse();
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

        intake.setPower(2*intakePower);

        // Power the Wobble Goal Arm
        double maxSpeed = 0.7;

        if(bumperLeftHit){
            if(currentPositionIndex < armPositions.length - 1) currentPositionIndex++;
            else currentPositionIndex = 0;

            double pos = armPositions[currentPositionIndex];

            if(currentPositionIndex == 0){
                // Make position[0] 0 degrees after first usage (not really sure why; Eva wanted this)
                pos -= offset - 10;
                moveForward = true;
            }

            arm.setPosition(pos, maxSpeed);
        }
        // (Optional) custom controller
        else{
            double error = arm.getTargetPosition() - arm.getPosition();
            double threshold = 2;

            if(Math.abs(error) < threshold && moveForward){
                arm.setPosition(armPositions[currentPositionIndex], maxSpeed);
                moveForward = false;
            }

            arm.setPower(DcMotorX.getControlledSpeed(maxSpeed, threshold, error, true));
        }

        // Move the claw servo
        if(bumperRightHit) claw.setAngle(claw.getAngle() > 0 ? 0 : 100);


        // Move the launcher servo
        double timeElapsed = System.currentTimeMillis() - lastTimeHit;

        if((aHit || (a && timeElapsed > 450)) && launcher.getPower() > 0){
            lastTimeHit = System.currentTimeMillis();
            flipperClosed = false;
            flipper.setAngle(25);
        }else if(timeElapsed > 200 && !flipperClosed){
            flipperClosed = true;
            flipper.setAngle(50);
        }


        // Turn intake wheels on/off
        double intakeWheelPower = intakeWheels.getPower();

        if(xHit){
            if(intakeWheelPower == 0){
                intakeWheels.setPower(1);
            }else{
                intakeWheels.setPower(0);
            }
        }else if(b && intakeWheelPower <= 0){
            intakeWheels.setPower(-1);
        }else if(intakeWheelPower < 0){ //Reverse mode should only run when b is held
            intakeWheels.setPower(0);
        }

        // Turn launcher flywheel on/off
        if(yHit) launcher.setPower(launcher.getPower() == 0 ? launcherSpeed : 0);

        if(dpadRightHit){
            launcherSpeed = 0.68;
        }else if(dpadLeftHit){
            launcherSpeed = 0.6;
        }

        if((dpadRightHit || dpadLeftHit) && launcher.getPower() > 0){
            launcher.setPower(launcherSpeed);
        }
        telemetry.addData("Launcher speed", 100*launcherSpeed + "%");

        // Save button states
        lastButtons1.update(a, b, x, y);
        lastDpads1.update(dpadUp, dpadDown, dpadRight, dpadLeft);
        lastBumpers1.update(bumperRight, bumperLeft);

        telemetry.addData("x", drivetrain.positionTracker.x);
        telemetry.addData("y", drivetrain.positionTracker.y);
        telemetry.addData("Heading", drivetrain.positionTracker.phi);
    }

    private double rateCurve(double input, double rate){
        return Math.pow(Math.abs(input),rate)*((input>0)?1:-1);
    }

    private double getClosestAngleToZero(double angle){
        return Math.round(angle/(2*Math.PI))*2*Math.PI;
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
