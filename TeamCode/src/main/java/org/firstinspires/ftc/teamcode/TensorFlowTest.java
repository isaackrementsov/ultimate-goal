package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.api.Robot;

import java.util.List;

public class TensorFlowTest extends LinearOpMode {

    private Robot bot;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private final String VUFORIA_KEY = "AY3aN3z/////AAABmUIe2Kd1wEt0nkr2MAal4OQiiEFWa3aLCHRnFBO1wd2HDT+GFXOTpcrhqEiZumOHpODdyVc55cYOiTSxpPrN+zfw7ZYB8X5z3gRLRIhPj4BJLD0/vPTKil7rDPSluUddISeCHL1HzPdIfiZiG/HQ89vhBdLfrWpngKLF4tH4FB4YWdKZu5J9EBtVTlXqR1OUXVTM3p9DepM9KukrVxMESF/ve+RYix7UXMO5qbljnc/LjQdplFO8oX4ztEe3aMXN14GadXggrfW+0m3nUmT8rXNTprc62LR1v0RbB4L+0QWfbgSDRyeMdBrvg8KIKLb1VFVrgUecbYBtHTTsLZALnU7oOOARnfGdtHC0aG3FAGxg";
    private final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private final String STONE_LABEL = "Stone";
    private final String SKYSTONE_LABEL = "SkyStone";
    private final String FOUNDATION_LABEL = "Foundation";

    public void runOpMode(){
        this.bot = new Robot(hardwareMap, telemetry);
        initVuforia();

        if(ClassFactory.getInstance().canCreateTFObjectDetector()){
            initTfod();
        }else{
            telemetry.addData("Error:", "This device is not compatible with TFOD");
        }

        waitForStart();

        if(tfod != null){
            tfod.activate();

            while(!isStopRequested()){
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if(updatedRecognitions != null){
                    for(Recognition recognition : updatedRecognitions){
                        telemetry.addData("Label: ", recognition.getLabel());
                        telemetry.addData("Bottom: ", recognition.getBottom());
                        telemetry.addData("Left: ", recognition.getLeft());
                    }
                }

                telemetry.update();
            }

            tfod.shutdown();
        }
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
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, STONE_LABEL, SKYSTONE_LABEL, FOUNDATION_LABEL);
    }

}
