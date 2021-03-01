package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.complex.Complex;
import org.apache.commons.math3.transform.DftNormalization;
import org.apache.commons.math3.transform.TransformType;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.api.ControlledDrivetrain;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.apache.commons.math3.transform.FastFourierTransformer;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;

@Autonomous
public class PositionControlCalibration extends LinearOpMode {
    // Encoder wheel information
    private double circumference = 15.71;
    private int ticksPerRev = 8192;
    // Robot width and back encoder wheel ticks per degree
    private double width = 40.8;
    private double backDistancePerDegree = 41.577/(2*Math.PI);

    // Size of a tile on the field (in cm)
    private final double TILE_SIZE = 60.95;

    private CalibrationMode mode = CalibrationMode.PHI;
    private double Ku = 40;
    private double[] Kua = new double[]{/*9*/0,/*7*/0,1.4};

    private ArrayList<Double> E = new ArrayList<>();
    private ArrayList<Double> t = new ArrayList<>();
    private FastFourierTransformer transformer = new FastFourierTransformer(DftNormalization.STANDARD);

    private ControlledDrivetrain drivetrain;

    public void runOpMode(){
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
        Odometry positionTracker = new Odometry(wheelR, wheelL, wheelB, 50, backDistancePerDegree, width, 0, 0, 0);

        // Instantiate the PID-controlled drivetrain
        drivetrain = new ControlledDrivetrain(mRF, mLF, mRB, mLB, positionTracker);
        drivetrain.reverse();
        drivetrain.telemetry = telemetry;
        initializeCoefficients();

        Thread drivetrainThread = new Thread(drivetrain);

        waitForStart();

        drivetrainThread.start();
        drivetrain.setActive(true);
        setTestPosition();

        long t0 = System.currentTimeMillis();
        int sampleEvery = 100;
        long samplingTime = 5000;

        while(!isStopRequested() && System.currentTimeMillis() - t0 < samplingTime){
            if(System.currentTimeMillis() % sampleEvery == 0){
                addData();
            }
        }
        drivetrain.setActive(false);
        drivetrain.setBrake(true);
        drivetrain.stop();

        Exception e = saveCSV(sampleEvery);
        if(e != null) telemetry.addData("Error saving CSV:", e);
        telemetry.addData("Ku", Ku);
        //telemetry.addData("Tu", computePeriod(samplingRate));
        telemetry.update();

        while(!isStopRequested());

        drivetrain.stop();
        drivetrain.setActive(false);
        drivetrain.stopController();
    }

    public void initializeCoefficients(){
        drivetrain.Kd = new double[]{0,0,25};
        drivetrain.Ki = new double[]{0,0,/*4.75*/0};

        if(mode == CalibrationMode.X) drivetrain.Kp = new double[]{Ku,0,0};
        if(mode == CalibrationMode.Y) drivetrain.Kp = new double[]{0,Ku,0};
        if(mode == CalibrationMode.PHI) drivetrain.Kp = new double[]{0,0,Ku};
        if(mode == CalibrationMode.ALL) drivetrain.Kp = Kua;
    }

    public void addData(){
        if(mode == CalibrationMode.X) E.add(drivetrain.ExL);
        if(mode == CalibrationMode.Y) E.add(drivetrain.EyL);
        if(mode == CalibrationMode.PHI) E.add(drivetrain.EphiL);
    }

    public void setTestPosition(){
        if(mode == CalibrationMode.X) drivetrain.setPosition(TILE_SIZE, 0, 0);
        if(mode == CalibrationMode.Y) drivetrain.setPosition(0, TILE_SIZE, 0);
        if(mode == CalibrationMode.PHI) drivetrain.setPosition(0, 0, 2*Math.PI);
        if(mode == CalibrationMode.ALL) drivetrain.setPosition(/*1.3*TILE_SIZE*/0, 0, 2*Math.PI);
    }

    public double computePeriod(int samplingRate){
        int n = E.size();
        double[] data = getPrimitive(E);

        Complex[] frequencies = transformer.transform(data, TransformType.FORWARD);
        double[] max = new double[]{0,0};
        int[] indices = new int[]{0,0};

        for(int i = 0; i < n; i++){
            double k = frequencies[i].getReal();

            if(k > max[1]){
                if(k > max[0]){
                    max[0] = k;
                    indices[0] = i;
                }else{
                    max[1] = k;
                    indices[1] = i;
                }
            }
        }

        int freqIndex = Math.min(indices[0], indices[1]);
        double freq = 1.0*samplingRate*freqIndex/(n*1.0);

        return 1/freq;
    }

    public static double[] getPrimitive(ArrayList<Double> list){
        double[] ret = new double[list.size()];

        for(int i = 0; i < ret.length; i++){
            ret[i] = list.get(i).doubleValue();
        }

        return ret;
    }

    private Exception saveCSV(int sampleEvery){
        try {
            File file = AppUtil.getInstance().getSettingsFile("PIDtuning.csv");
            FileWriter csvWriter = new FileWriter(file);

            csvWriter.append("T");
            csvWriter.append(",");
            csvWriter.append("B");
            csvWriter.append("\n");

            for(int i = 0; i < E.size(); i++){
                csvWriter.append(E.get(i).toString());
                csvWriter.append(",");
                csvWriter.append(String.valueOf(i*sampleEvery));
                csvWriter.append("\n");
            }

            csvWriter.flush();
            csvWriter.close();

            telemetry.addData("Successfully saved csv", "");

            return null;
        }catch (Exception e){
            telemetry.addData("Error Saving File", e);

            return e;
        }
    }

    private static enum CalibrationMode { X,Y,PHI,ALL }
}
