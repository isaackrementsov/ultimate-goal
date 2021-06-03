package org.firstinspires.ftc.teamcode.api;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.List;

public class TensorFlowX {

    private String vuforiaKey;
    private VuforiaLocalizer.CameraDirection camera;

    private String tfodModelAsset;
    private int tfodMonitorViewID;
    private String[] labels;

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public TensorFlowX(String tfodModelAsset, String vuforiaKey, VuforiaLocalizer.CameraDirection camera, String[] labels, HardwareMap hardwareMap) throws Exception {
        this.tfodModelAsset = tfodModelAsset;
        this.vuforiaKey = vuforiaKey;
        this.camera = camera;
        this.labels = labels;

        tfodMonitorViewID = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
        );

        init();
    }

    private void initVuforia(){
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();

        params.vuforiaLicenseKey = vuforiaKey;
        params.cameraDirection = camera;

        vuforia = ClassFactory.getInstance().createVuforia(params);
    }

    private void initTfod(){
        TFObjectDetector.Parameters params = new TFObjectDetector.Parameters(tfodMonitorViewID);
        params.minResultConfidence = 0.8f;

        tfod = ClassFactory.getInstance().createTFObjectDetector(params, vuforia);
        tfod.loadModelFromAsset(tfodModelAsset, labels);
    }

    private void init() throws Exception {
        initVuforia();
        initTfod();

        if(tfod != null){
            tfod.activate();
        }else{
            throw new Exception("This device is not compatible with TFOD");
        }
    }

    public void shutdown(){
        tfod.shutdown();
    }

    public Recognition recognize(String label){
        Recognition matched = null;
        List<Recognition> updated = tfod.getUpdatedRecognitions();

        if(updated != null){
            List<Recognition> recognitions = tfod.getRecognitions();
            if(recognitions != null) updated.addAll(recognitions);

            for(Recognition recognition : updated){
                if(recognition.getLabel().equals(label)){
                    matched = recognition;
                    break;
                }
            }
        }else{
            matched = null;
        }

        return matched;
    }

    public List<Recognition> recognizeAll(String label){
        List<Recognition> matched = new ArrayList<>();
        List<Recognition> updated = tfod.getUpdatedRecognitions();

        List<Recognition> recognitions = tfod.getRecognitions();
        if(recognitions != null) updated.addAll(recognitions);

        for(Recognition recognition : updated){
            if(recognition.getLabel().equals(label)){
                matched.add(recognition);
            }
        }

        return matched;
    }

    public List<Recognition> getAll(){
        List<Recognition> recs = tfod.getUpdatedRecognitions();
        recs.addAll(tfod.getRecognitions());
        return recs;
    }

}
