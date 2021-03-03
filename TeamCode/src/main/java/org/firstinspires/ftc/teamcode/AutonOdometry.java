package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.api.ControlledDrivetrain;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.firstinspires.ftc.teamcode.api.Robot;

import java.util.List;

@Autonomous
public class AutonOdometry extends LinearOpMode {

    private Robot bot;

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
    private double offset = -60;
    // Where the robot starts (in cm from the right wall)
    private double x0 = 112.395;
    private double y0 = 0;
    private double phi0 = 0;
    private double DETECTION_POS = 0.5*TILE_SIZE;
    private double SHOOTING_POS = 0.55*TILE_SIZE;

    @Override
    public void runOpMode() throws InterruptedException {
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
        drivetrain.reverse();
        drivetrain.telemetry = telemetry;
        // Run it in a separate thread
        Thread drivetrainThread = new Thread(drivetrain);

        this.bot = new Robot(hardwareMap, telemetry);

        // Register the Wobble Arm
        bot.addLimitedMotor("arm", "armLimit", "armLimit", 360, 3*288, true);

        bot.addDcMotor("launcher", false);
        bot.runAtConstantVelocity("launcher");

        bot.addServo("flipper");
        bot.rotateServo("flipper", 100, 0);

        bot.addServo("claw", 270, 180, 0);
        bot.rotateServo("claw", 0, 0);

        bot.initCV(
                VUFORIA_KEY,
                VuforiaLocalizer.CameraDirection.BACK,
                TFOD_MODEL_ASSET,
                new String[]{QUAD_LABEL, SINGLE_LABEL}
        );

        waitForStart();

        // Start the thread
        drivetrainThread.start();


        // Zero arm position
        bot.resetLimitedMotor("arm", 0.2);

        if(bot.isCVReady){
            // Determine where to deliver the Wobble Goal by looking at the starter stack
            // Give the bot 2 seconds to look
            char zone = determineTargetZone(3000);

            telemetry.addData("Target Zone: ", zone);
            telemetry.update();

            // Park the robot on the launch line based on where it is after driving to target zone
            driveToTargetZone(zone);
            goBehindLine();

            // Shoot 3 rings
            shoot();

            bot.shutDownCV();

            drivetrain.setBrake(true);
            drivetrain.stop();
            drivetrain.setActive(false);
            drivetrain.stopController();
        }
    }

    private char determineTargetZone(int waitTime){
        // Number of rings in the starter stack
        int stackedRings = 0;

        // Drive to detection area
        setPositionAndWait(TILE_SIZE, DETECTION_POS,0);

        // Define time that the robot should be done looking at the stack
        long t = System.currentTimeMillis();
        long end = t + waitTime;

        // Continue checking for rings until the time runs out or stacked rings are detected
        while(System.currentTimeMillis() < end && stackedRings == 0 && !isStopRequested()){
            // Get updated object recognition data from TensorFlow
            try {
                Recognition quad = bot.recognize(QUAD_LABEL);
                Recognition single = bot.recognize(SINGLE_LABEL);

                // Determine number of rings based on stack type detected
                if(quad != null){
                    stackedRings = 4;
                }else if(single != null){
                    stackedRings = 1;
                }else{
                    stackedRings = 0;
                }
            }catch(Exception e){
                return 0;
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

    private void driveToTargetZone(char zone){
        // Shift outward from the starter stack to avoid hitting it
        setPositionAndWait(2*TILE_SIZE, 2*TILE_SIZE, 0);

        switch(zone){
            case 'a':
                setPositionAndWait(TILE_SIZE, 3*TILE_SIZE, 0);
            case 'b':
                setPositionAndWait(2*TILE_SIZE, 4*TILE_SIZE, 0);
            case 'c':
                setPositionAndWait(TILE_SIZE, 5*TILE_SIZE, 0);
        }

        dropWobbleGoal();
    }

    private void dropWobbleGoal(){
        // Drop and release the wobble goal
        bot.moveDcMotor("arm", -90 + offset, 0.7, false);
        bot.rotateServo("claw", 100, 250);
        // Raise the arm
        bot.moveToStaticPosition("arm", 0, 0.7, false);
    }

    private void goBehindLine(){
        setPositionAndWait(TILE_SIZE, 2*TILE_SIZE, 0);
    }

    private void shoot(){
        bot.moveDcMotor("launcher", 0.7);
        bot.waitMillis(1000);

        for(int i = 0; i < 3; i++){
            bot.rotateServo("flipper", 160, 250);
            bot.rotateServo("flipper", 100, 500);
        }

        bot.moveDcMotor("launcher", 0);

        setPositionAndWait(TILE_SIZE, 3*TILE_SIZE, 0);
    }

    private void setPositionAndWait(double x, double y, double phi){
        drivetrain.setActive(true);
        // Robot is facing in reverse and x-coordinates are inverted, so use x,-y
        drivetrain.setPosition(x, -y, phi);

        try {
            Thread.sleep(50);
        }catch(Exception e){ }
        while(!isStopRequested() && drivetrain.isBusy());

        drivetrain.stop();
        drivetrain.setActive(false);
    }
}
