// TODO: CREATE MORE OPMODES FOR EACH TEAM STARTING POS!!!
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.api.Robot;

import java.util.List;

@Autonomous
public class AutonDraft extends LinearOpMode {

    private Robot bot;

    private final String VUFORIA_KEY = "AY3aN3z/////AAABmUIe2Kd1wEt0nkr2MAal4OQiiEFWa3aLCHRnFBO1wd2HDT+GFXOTpcrhqEiZumOHpODdyVc55cYOiTSxpPrN+zfw7ZYB8X5z3gRLRIhPj4BJLD0/vPTKil7rDPSluUddISeCHL1HzPdIfiZiG/HQ89vhBdLfrWpngKLF4tH4FB4YWdKZu5J9EBtVTlXqR1OUXVTM3p9DepM9KukrVxMESF/ve+RYix7UXMO5qbljnc/LjQdplFO8oX4ztEe3aMXN14GadXggrfW+0m3nUmT8rXNTprc62LR1v0RbB4L+0QWfbgSDRyeMdBrvg8KIKLb1VFVrgUecbYBtHTTsLZALnU7oOOARnfGdtHC0aG3FAGxg";
    private final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private final String QUAD_LABEL = "Quad";
    private final String SINGLE_LABEL = "Single";

    private final double TILE_SIZE = 60.96;

    private double LOW_POWER = 0.5;
    private double HIGH_POWER = 1;

    // Arm starting position
    private double offset = -60;
    // Where the robot starts (in tiles from the right wall)
    private double STARTING_POS = 2.5;
    private double DETECTION_POS = 0.6;
    private double SHOOTING_POS = 0.55;

    @Override
    public void runOpMode() throws InterruptedException {
        this.bot = new Robot(hardwareMap, telemetry);
        bot.addDrivetrain(
                new String[]{"mRF", "mLF", "mRB", "mLB"},
                new double[]{31.42, 31.42, 31.42, 31.42},
                new double[]{767.2, 767.2, 767.2, 767.2},
                1,
                true
        );
        // Reverse the drivetrain so that the camera can be mounted on the back without changing driving plans
        bot.reverseDrivetrain();
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

        // Zero arm position
        bot.resetLimitedMotor("arm", 0.2);

        if(bot.isCVReady){
            // Determine where to deliver the Wobble Goal by looking at the starter stack
            // Give the bot 2 seconds to look
            char zone = determineTargetZone(3000);

            telemetry.addData("Target Zone: ", zone);
            telemetry.update();

            // Park the robot on the launch line based on where it is after driving to target zone
            double[] directions = driveToTargetZone(zone);
            goBehindLine(directions);

            // Shoot 3 rings
            shoot();

            bot.shutDownCV();
        }
    }

    private char determineTargetZone(int waitTime){
        // Number of rings in the starter stack
        int stackedRings = 0;

        bot.drive(0.2, DETECTION_POS*TILE_SIZE, Robot.Direction.FORWARD);

        // Define time that the robot should be done looking at the stack
        long t = System.currentTimeMillis();
        long end = t + waitTime;

        // Continue checking for rings until the time runs out or stacked rings are detected
        while(System.currentTimeMillis() < end && stackedRings == 0){
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

    private double[] driveToTargetZone(char zone){
        double driveBackward = 0;
        double driveLeft = 0;

        // Shift outward from the starter stack to avoid hitting it
        bot.drive(LOW_POWER, TILE_SIZE*(3 - STARTING_POS), Robot.Direction.LEFT);
        bot.drive(LOW_POWER, TILE_SIZE*(3 - DETECTION_POS), Robot.Direction.FORWARD);

        switch(zone){
            case 'a':
                driveBackward = 0.4;
                driveLeft = 1.2;

                bot.drive(LOW_POWER, driveLeft*TILE_SIZE, Robot.Direction.RIGHT);

                break;
            case 'b':
                driveBackward = 1.2;
                driveLeft = 0.2;

                bot.drive(LOW_POWER, driveBackward*TILE_SIZE, Robot.Direction.FORWARD);
                bot.drive(LOW_POWER, driveLeft*TILE_SIZE, Robot.Direction.RIGHT);

                break;
            case 'c':
                driveBackward = 2;
                driveLeft = 1.2;

                bot.drive(LOW_POWER, driveBackward*TILE_SIZE, Robot.Direction.FORWARD);
                bot.drive(LOW_POWER, driveLeft*TILE_SIZE, Robot.Direction.RIGHT);

                break;
        }

        dropWobbleGoal();

        // Directions to go behind line before shooting
        return new double[]{driveBackward + SHOOTING_POS, driveLeft - 0.45};
    }

    private void dropWobbleGoal(){
        bot.stop();

        // Drop and release the wobble goal
        bot.moveDcMotor("arm", -90 + offset, 0.7, false);
        bot.rotateServo("claw", 100, 250);
        // Raise the arm
        bot.moveLimitedMotor("arm", 0.7);
    }

    private void goBehindLine(double[] directions){
        double driveBackward = directions[0];
        double driveLeft = directions[1];

        bot.drive(LOW_POWER, TILE_SIZE*driveBackward, Robot.Direction.BACKWARD);
        bot.drive(LOW_POWER, TILE_SIZE*driveLeft, Robot.Direction.LEFT);

        bot.stop();
    }

    private void shoot(){
        bot.moveDcMotor("launcher", 0.7);
        bot.waitMillis(1000);

        for(int i = 0; i < 3; i++){
            bot.rotateServo("flipper", 160, 250);
            bot.rotateServo("flipper", 100, 500);
        }

        bot.moveDcMotor("launcher", 0);
        bot.drive(LOW_POWER, (SHOOTING_POS + 0.1)*TILE_SIZE, Robot.Direction.FORWARD);
    }
}
