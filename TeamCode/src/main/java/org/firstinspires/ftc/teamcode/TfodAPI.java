package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.api.TensorFlowX;

@Autonomous
public class TfodAPI extends LinearOpMode {

    private TensorFlowX tfod;
    private final String VUFORIA_KEY = "AY3aN3z/////AAABmUIe2Kd1wEt0nkr2MAal4OQiiEFWa3aLCHRnFBO1wd2HDT+GFXOTpcrhqEiZumOHpODdyVc55cYOiTSxpPrN+zfw7ZYB8X5z3gRLRIhPj4BJLD0/vPTKil7rDPSluUddISeCHL1HzPdIfiZiG/HQ89vhBdLfrWpngKLF4tH4FB4YWdKZu5J9EBtVTlXqR1OUXVTM3p9DepM9KukrVxMESF/ve+RYix7UXMO5qbljnc/LjQdplFO8oX4ztEe3aMXN14GadXggrfW+0m3nUmT8rXNTprc62LR1v0RbB4L+0QWfbgSDRyeMdBrvg8KIKLb1VFVrgUecbYBtHTTsLZALnU7oOOARnfGdtHC0aG3FAGxg";
    private final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private final String QUAD_LABEL = "Quad";
    private final String SINGLE_LABEL = "Single";

    public void runOpMode(){
        try {
            tfod = new TensorFlowX(
                    TFOD_MODEL_ASSET,
                    VUFORIA_KEY,
                    VuforiaLocalizer.CameraDirection.BACK,
                    new String[]{QUAD_LABEL, SINGLE_LABEL},
                    hardwareMap
            );
        }catch (Exception e){
            telemetry.addData("Error initializing TensorFlow", e);
        }

        waitForStart();

        while(!isStopRequested()){
            Recognition quad = tfod.recognize(QUAD_LABEL);
            Recognition single = tfod.recognize(SINGLE_LABEL);

            if(quad != null){
                telemetry.addData("Quad recognized with confidence", quad.getConfidence());
                telemetry.update();
            }

            if(single != null){
                telemetry.addData("Single recognized with confidence", single.getConfidence());
                telemetry.update();
            }

        }

        tfod.shutdown();
    }

}
