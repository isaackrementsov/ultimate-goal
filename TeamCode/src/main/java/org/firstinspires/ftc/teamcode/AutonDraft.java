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

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private final String VUFORIA_KEY = "AY3aN3z/////AAABmUIe2Kd1wEt0nkr2MAal4OQiiEFWa3aLCHRnFBO1wd2HDT+GFXOTpcrhqEiZumOHpODdyVc55cYOiTSxpPrN+zfw7ZYB8X5z3gRLRIhPj4BJLD0/vPTKil7rDPSluUddISeCHL1HzPdIfiZiG/HQ89vhBdLfrWpngKLF4tH4FB4YWdKZu5J9EBtVTlXqR1OUXVTM3p9DepM9KukrVxMESF/ve+RYix7UXMO5qbljnc/LjQdplFO8oX4ztEe3aMXN14GadXggrfW+0m3nUmT8rXNTprc62LR1v0RbB4L+0QWfbgSDRyeMdBrvg8KIKLb1VFVrgUecbYBtHTTsLZALnU7oOOARnfGdtHC0aG3FAGxg";
    private final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private final String RING_LABEL = "Stone";

    private final double TILE_SIZE = 60.96;

    private double LOW_POWER = 0.2;
    private double HIGH_POWER = 1;

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

        initVuforia();

        if(ClassFactory.getInstance().canCreateTFObjectDetector()){
            initTfod();
        }else{
            telemetry.addData("Error:", "This device is not compatible with TFOD");
        }

        waitForStart();

        if(tfod != null){
            tfod.activate();

            // Determine where to deliver the Wobble Goal by looking at the starter stack
            // Give the bot 2 seconds to look
            char zone = determineTargetZone(2000);

            telemetry.addData("Target Zone: ", zone);
            telemetry.update();

            // Park the robot on the launch line based on where it is after driving to target zone
            parkOnLine(driveToTargetZone(zone));

            tfod.shutdown();
        }
    }

    private char determineTargetZone(int waitTime){
        // Number of rings in the starter stack
        int stackedRings = 0;

        // Define time that the robot should be done looking at the stack
        long t = System.currentTimeMillis();
        long end = t + waitTime;

        // Continue checking for rings until the time runs out or stacked rings are detected
        while(System.currentTimeMillis() < end && stackedRings == 0){
            // Get updated object recognition data from TensorFlow
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if(updatedRecognitions != null){
                telemetry.addData("Recognition!", "");
                telemetry.update();
                // For each ring seen by TensorFlow, increase the starter stack size
                for(Recognition object: updatedRecognitions){
                    if(object.getLabel().equals(RING_LABEL)){
                        stackedRings++;
                    }
                }
            }
        }

        // Determine target zone based on starter stack
        switch(stackedRings){
            case 0:
                return 'a';
            case 1:
                return 'b';
            case 4:
                return 'c';
            // If the stack size is not 0, 1, or 4, the target zone is unknown (alert if this happens)
            default:
                return 'd';
        }
    }

    private int[] driveToTargetZone(char zone){
        int driveBackward = 0;
        int driveLeft = 0;

        bot.drive(LOW_POWER, TILE_SIZE*3.5, Robot.Direction.FORWARD);

        switch(zone){
            case 'a':
                bot.drive(LOW_POWER, TILE_SIZE*1.5, Robot.Direction.RIGHT);

                driveBackward = 0;
                driveLeft = 3;

                break;
            case 'b':
                bot.drive(LOW_POWER, TILE_SIZE, Robot.Direction.FORWARD);
                bot.drive(LOW_POWER, TILE_SIZE*0.5, Robot.Direction.RIGHT);

                driveBackward = 1;
                driveLeft = 2;

                break;
            case 'c':
                bot.drive(LOW_POWER, TILE_SIZE*2, Robot.Direction.FORWARD);
                bot.drive(LOW_POWER, TILE_SIZE*1.5, Robot.Direction.RIGHT);

                driveBackward = 2;
                driveLeft = 3;

                break;
        }

        dropWobbleGoal();

        return new int[]{driveBackward, driveLeft};
    }

    private void dropWobbleGoal(){
        bot.stop();
    }

    private void parkOnLine(int[] directions){
        int driveBackward = directions[0];
        int driveLeft = directions[1];

        bot.drive(LOW_POWER, TILE_SIZE*driveLeft, Robot.Direction.LEFT);
        bot.drive(LOW_POWER, TILE_SIZE*driveBackward, Robot.Direction.BACKWARD);

        bot.stop();
    }

    private void initVuforia(){
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();

        params.vuforiaLicenseKey = VUFORIA_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(params);
    }

    private void initTfod(){
        int tfodMonitorViewID = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
        );

        TFObjectDetector.Parameters params = new TFObjectDetector.Parameters(tfodMonitorViewID);

        tfod = ClassFactory.getInstance().createTFObjectDetector(params, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, RING_LABEL);
    }

}
