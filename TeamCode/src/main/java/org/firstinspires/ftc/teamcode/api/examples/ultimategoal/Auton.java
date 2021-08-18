package org.firstinspires.ftc.teamcode.api.examples.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.api.ControlledDrivetrain;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.LimitedMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.firstinspires.ftc.teamcode.api.ServoX;
import org.firstinspires.ftc.teamcode.api.TensorFlowX;

@Autonomous
public class Auton extends LinearOpMode {

    // Odometry parameters
    private int ticksPerRev = 8192;
    private double circumference = 15.71;
    private double width = 40.8;
    private double backDistancePerRadian = -41.577/(2*Math.PI);

    private final String VUFORIA_KEY = "AY3aN3z/////AAABmUIe2Kd1wEt0nkr2MAal4OQiiEFWa3aLCHRnFBO1wd2HDT+GFXOTpcrhqEiZumOHpODdyVc55cYOiTSxpPrN+zfw7ZYB8X5z3gRLRIhPj4BJLD0/vPTKil7rDPSluUddISeCHL1HzPdIfiZiG/HQ89vhBdLfrWpngKLF4tH4FB4YWdKZu5J9EBtVTlXqR1OUXVTM3p9DepM9KukrVxMESF/ve+RYix7UXMO5qbljnc/LjQdplFO8oX4ztEe3aMXN14GadXggrfW+0m3nUmT8rXNTprc62LR1v0RbB4L+0QWfbgSDRyeMdBrvg8KIKLb1VFVrgUecbYBtHTTsLZALnU7oOOARnfGdtHC0aG3FAGxg";
    private final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private final String QUAD_LABEL = "Quad";
    private final String SINGLE_LABEL = "Single";

    private ControlledDrivetrain drivetrain;

    private final double TILE_SIZE = 60.96;

    // Arm starting position
    private final double offset = -60;
    // Where the robot starts (in cm from the right wall)
    private final double x0 = 112.395;
    private final double y0 = 0;
    private final double phi0 = /*-0.25*Math.PI/180*/0;
    private final double[] DETECTION_POS = new double[]{1.6*TILE_SIZE, 0.8*TILE_SIZE - 5};

    //private final double[] AVOID_STACK_POS = new double[]{3*TILE_SIZE, 2.2*TILE_SIZE};
    //private final double[] AVOID_STACK_THRESH = new double[]{4,4,3};

    private final double[] SHOOTING_POS = new double[]{100.2 + 2.5, 148 + 3};

    private final double[] POWER_SHOTS_X = new double[]{174.6, 160.2, 135.9};
    private final double[] POWER_SHOTS_Y = new double[]{146,150.3,153.47};

    private final double[] DROP_ZONES_X = new double[]{32.55 + 10 + 5 + 10, 103.1 + 8 + 5, 43.8 + 12 - 7};
    private final double[] DROP_ZONES_Y = new double[]{205.1 - 2, 258.9 + 1, 307.1 + 2.5};

    private final double WOBBLE_X = 89;
    private final double WOBBLE_Y = 42;

    private DcMotorX
        mRF,
        mLF,
        mRB,
        mLB,
        wheelR,
        wheelL,
        wheelB,
        launcher,
        intake,
        intakeWheels;

    private LimitedMotorX arm;

    private ServoX
        flipper,
        claw;

    private TensorFlowX tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get all of the drivetrain motors
        mRF = new DcMotorX(hardwareMap.dcMotor.get("mRF"));
        mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF"));
        mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB"));
        mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));

        // Get the odometry wheels
        wheelR = new DcMotorX(hardwareMap.dcMotor.get("mRB"), ticksPerRev, circumference);
        wheelL = new DcMotorX(hardwareMap.dcMotor.get("mLF"), ticksPerRev, circumference);
        wheelB = new DcMotorX(hardwareMap.dcMotor.get("mRF"), ticksPerRev, circumference);

        // Create an odometry instance for the drivetrain
        Odometry positionTracker = new Odometry(wheelR, wheelL, wheelB, 50, backDistancePerRadian, width, x0, y0, phi0);

        // Instantiate the PID-controlled drivetrain
        drivetrain = new ControlledDrivetrain(mRF, mLF, mRB, mLB, positionTracker);
        drivetrain.reverse();
        drivetrain.telemetry = telemetry;

        // Register the Wobble Arm
        arm = new LimitedMotorX(hardwareMap.dcMotor.get("arm"), 3*288, 360);
        arm.setLowerLimit(hardwareMap.touchSensor.get("armLimit"));
        arm.setBrake(true);

        launcher = new DcMotorX(hardwareMap.dcMotor.get("launcher"));
        launcher.controlVelocity();

        flipper = new ServoX(hardwareMap.servo.get("flipper"));
        flipper.setAngle(50);

        intake = new DcMotorX(hardwareMap.dcMotor.get("intake"));
        intake.setBrake(true);

        intakeWheels = new DcMotorX(hardwareMap.dcMotor.get("intakeWheels"));

        claw = new ServoX(hardwareMap.servo.get("claw"), 270, 180);
        claw.setAngle(0);

        try {
            tfod = new TensorFlowX(
                    TFOD_MODEL_ASSET,
                    VUFORIA_KEY,
                    VuforiaLocalizer.CameraDirection.BACK,
                    new String[]{QUAD_LABEL, SINGLE_LABEL},
                    hardwareMap
            );
        }catch(Exception e){
            telemetry.addData("Error initializing TensorFlow", e);
        }

        telemetry.addData("Done initializing", "");
        telemetry.update();

        waitForStart();

        // Run it in a separate thread
        Thread drivetrainThread = new Thread(drivetrain);
        // Start the thread
        drivetrainThread.start();

        // Zero arm position
        arm.reset();
        arm.goToPosition(-33, 1);

        // Determine where to deliver the Wobble Goal by looking at the starter stack
        // Give the bot 2 seconds to look
        char zone = determineTargetZone(6000);

        telemetry.addData("Target Zone: ", zone);
        telemetry.update();

        // Shoot 3 rings
        shootHighGoal(false);
        // Drop wobble #1
        driveToTargetZone(zone);

        // Drive to wobble #2
        pickUpWobble(zone);

        if (zone == 'c') {
            shootHighGoal(true);
        }

        // Drop wobble #2
        driveToTargetZone(zone, 10, 15);

        // Park the robot on the launch line based on where it is after driving to target zone
        if(zone != 'b'){
            parkOverLaunchLine(zone);
        }

        tfod.shutdown();

        drivetrain.setBrake(true);
        drivetrain.stop();
        drivetrain.setActive(false);
        drivetrain.stopController();
    }

    private char determineTargetZone(long waitTime){
        // Number of rings in the starter stack
        int stackedRings = 0;

        // Drive to detection area
        drivetrain.setActive(true);
        setPosition(DETECTION_POS[0], DETECTION_POS[1],0);

        // Define time that the robot should be done looking at the stack
        long start = System.currentTimeMillis();

        long minTime = 1000;

        // Continue checking for rings until the time runs out or stacked rings are detected
        while(((System.currentTimeMillis() - start) < waitTime && stackedRings == 0) || (System.currentTimeMillis() - start) < minTime && stackedRings == 0 && !isStopRequested()){

            // Get updated object recognition data from TensorFlow
            try {
                Recognition quad = tfod.recognize(QUAD_LABEL);
                Recognition single = tfod.recognize(SINGLE_LABEL);

                // Determine number of rings based on stack type detected
                if(quad != null){
                    stackedRings = 4;
                }else if(single != null){
                    stackedRings = 1;
                }else{
                    stackedRings = 0;
                }
            }catch(Exception e){
                telemetry.addData("Exception", e);
                return 'a';
            }
        }

        // Determine target zone based on starter stack
        switch(stackedRings){
            case 1:
                return 'b';
            case 4:
                return 'c';
            default: // Equivalent to `case 0:`
                return 'a';
        }
    }

    private void driveToTargetZone(char zone, double offsetX, double offsetY) throws InterruptedException {
        switch(zone){
            case 'a':
                setPositionAndWait(DROP_ZONES_X[0] + offsetX, DROP_ZONES_Y[0] - offsetY, 0);
                break;
            case 'b':
                setPositionAndWait(DROP_ZONES_X[1] + offsetX, DROP_ZONES_Y[1] - offsetY, 0);
                break;
            case 'c':
                setPositionAndWait(DROP_ZONES_X[2] + offsetX*2.2, DROP_ZONES_Y[2] - offsetY/3.6, 0);
                break;
        }

        pauseDrivetrain(50);
        dropWobbleGoal(zone, offsetX != 0);

        startDrivetrain();
    }

    private void driveToTargetZone(char zone) throws InterruptedException {
        driveToTargetZone(zone, 0, 0);
    }

    private void dropWobbleGoal(char zone, boolean secondWobble) throws InterruptedException {
        // Drop and release the wobble goal
        arm.setPosition(-60 + offset, 1);
        claw.goToAngle( 100, 250);

        startDrivetrain();

        setPositionAndWait(drivetrain.positionTracker.x, -(drivetrain.positionTracker.y - 5), 0);
        pauseDrivetrain(50);

        // Raise the arm
        arm.goToPosition( -33, 1);

        if((zone == 'c' || zone == 'b') && secondWobble){
            extendRake();
        }
        while(!isStopRequested() && arm.getPosition() < -40);

        startDrivetrain();
    }

    private void shootHighGoal(boolean secondTime) throws InterruptedException {
        launcher.setVelocity(0.67);
        if(!secondTime) setPositionAndWait(SHOOTING_POS[0] + 12, SHOOTING_POS[1], 0, 3, 5, 0.1);
        setPositionAndWait(SHOOTING_POS[0], SHOOTING_POS[1], 0);

        for(int i = 0; i < 3; i++){
            flipper.goToAngle( 25, 250);
            flipper.goToAngle(50, 500);
        }

        launcher.setVelocity(0);
    }

    private void shootPowerShots() throws InterruptedException {
        // Power on the launcher
        launcher.setVelocity(0.6);
        Thread.sleep(200);

        for(int i = 0; i < POWER_SHOTS_X.length; i++){
            setPositionAndWait(POWER_SHOTS_X[i], POWER_SHOTS_Y[i], 0);
            fire();
        }

        launcher.setVelocity(0);
    }

    private void fire() throws InterruptedException {
        pauseDrivetrain(50);
        flipper.goToAngle(25, 250);
        flipper.goToAngle( 50, 500);
        startDrivetrain();
    }

    private void pickUpRings(){
        // Pick up some of the ring stack
        intakeWheels.setPower(1);

        setPositionAndWait(WOBBLE_X, WOBBLE_Y, 0);
        intakeWheels.setPower(0);
    }

    private void pickUpWobble(char zone) throws InterruptedException {
        // Whether or not the robot is going to drive over rings
        boolean ringStack = zone != 'a';

        // Pick up some of the ring stack
        if(ringStack) intakeWheels.setPower(1);

        setPositionAndWait(WOBBLE_X, WOBBLE_Y + 0.5*TILE_SIZE, 0);

        pauseDrivetrain(50);
        // Raise the arm
        arm.setPosition( -60 + offset, 1);
        // Avoid taking in a 4th ring
        startDrivetrain();

        // Drive to wobble
        setPositionAndWait(WOBBLE_X, WOBBLE_Y, 0);
        claw.goToAngle(100, 250);
        pauseDrivetrain(50);

        // Raise the arm
        claw.goToAngle(0, 200);
        arm.setPosition(-33, 1);
        while(!isStopRequested() && arm.getPosition() < -40);

        if(ringStack) intakeWheels.setPower(0);
        startDrivetrain();
    }

    private void extendRake(){
        intake.setPower(1);
        sleep(380);
        intake.setPower(0);
    }

    private void parkOverLaunchLine(char zone){
        if(zone == 'a'){
            setPosition(drivetrain.positionTracker.x, 3*TILE_SIZE, 0);
        }else{
            setPosition(drivetrain.positionTracker.x, 3*TILE_SIZE + 65, 0);
        }
    }

    private void setPositionAndWait(double x, double y, double phi, double xThresh, double yThresh, double phiThresh){
        // Robot is facing in reverse and x-coordinates are inverted, so use x,-y
        drivetrain.setPosition(x, -y, phi);
        sleep(50);
        while(!isStopRequested() && drivetrain.isBusy(xThresh, yThresh, phiThresh));

        drivetrain.stop();
    }

    private void setPositionAndWait(double x, double y, double phi){
        // Robot is facing in reverse and x-coordinates are inverted, so use x,-y
        drivetrain.setPosition(x, -y, phi);
        sleep(50);
        while(!isStopRequested() && drivetrain.isBusy());
        drivetrain.stop();
    }

    private void setPosition(double x, double y, double phi){
        drivetrain.setPosition(x,-y,phi);
    }

    private void sleep(int wait){
        try { Thread.sleep(wait); }catch(Exception e){ }
    }

    private void pauseDrivetrain(int wait){
        drivetrain.setActive(false);
        sleep(wait);
        drivetrain.stop();
    }

    private void startDrivetrain(){
        drivetrain.setActive(true);
    }
}
