package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.api.ControlledDrivetrain;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.firstinspires.ftc.teamcode.api.Robot;

@TeleOp
public class PIDCalibration extends OpMode {

    // Encoder wheel information
    private double circumference = 15.71;
    private int ticksPerRev = 8192;
    // Robot width and back encoder wheel ticks per degree
    private double width = 40.8;
    private double backDistancePerDegree = 41.577/(2*Math.PI);

    // Size of a tile on the field (in cm)
    private final double TILE_SIZE = 60.95;

    // Calibration parameters
    private double Kp = 40;
    private double Ki = 0;
    private double Kd = 25;

    // For editing coeffients in the OpMode
    private double increment = 1;
    private int editingIdx = 0;
    private String[] editingOptions = new String[]{"Kp", "Kd", "Ki"};

    // Coordinate to calibrate
    private PIDCalibration.CalibrationMode mode = PIDCalibration.CalibrationMode.PHI;
    // Test setpoint
    private double setpoint = 2*Math.PI;

    // Controller for tuning
    private ControlledDrivetrain drivetrain;
    // Saved button states
    private Robot.ButtonState lastButtons = new Robot.ButtonState();
    private Robot.DpadState lastDpads = new Robot.DpadState();

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
        Odometry positionTracker = new Odometry(wheelR, wheelL, wheelB, 50, backDistancePerDegree, width, 0, 0, 0);

        // Instantiate the PID-controlled drivetrain
        drivetrain = new ControlledDrivetrain(mRF, mLF, mRB, mLB, positionTracker);
        drivetrain.reverse();
        drivetrain.telemetry = telemetry;
        drivetrain.setActive(false);
        // Run it on a separate thread
        Thread drivetrainThread = new Thread(drivetrain);
        drivetrainThread.start();

        // Apply predefined settings
        initializeCoefficients();
        setPosition();
    }

    public void loop(){
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;

        boolean aHit = a && !lastButtons.a;
        boolean bHit = b && !lastButtons.b;
        boolean xHit = x && !lastButtons.x;
        boolean dpadUpHit = dpadUp && !lastDpads.dpad_up;
        boolean dpadDownHit = dpadDown && !lastDpads.dpad_down;

        // Stop position control
        if(bHit){
            drivetrain.setActive(false);
            drivetrain.setBrake(true);
            drivetrain.stop();
        }

        if(!drivetrain.getActive()){

            // Start/restart position control
            if(aHit){
                drivetrain.setBrake(false);
                drivetrain.setActive(true);
                drivetrain.positionTracker.reset();
            }

            // Select a coefficient to edit
            if(xHit){
                editingIdx = editingIdx < editingOptions.length - 1 ? editingIdx + 1 : 0;
            }

            // Increment selected coefficient
            if(dpadUpHit) editCoefficients(increment);
            else if(dpadDownHit) editCoefficients(-increment);

            // Log current coefficient values
            telemetry.addData("Currently editing", editingOptions[editingIdx]);
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
        }

        lastDpads.update(dpadUp, dpadDown, false, false);
        lastButtons.update(a, b, x, false);
    }

    public void stop(){
        drivetrain.setActive(false);
        drivetrain.setBrake(true);
        drivetrain.stop();
        drivetrain.stopController();
    }

    private void initializeCoefficients(){
        drivetrain.Kp = drivetrain.Kd = drivetrain.Ki = new double[]{0,0,0};
        int i = getModeIndex();

        drivetrain.Kp[i] = Kp;
        drivetrain.Kd[i] = Kd;
        drivetrain.Ki[i] = Ki;
    }

    private void editCoefficients(double change){
        if(editingIdx == 0) Kp += change;
        if(editingIdx == 1) Kd += change;
        if(editingIdx == 2) Ki += change;
    }

    private void setPosition(){
        double[] coords = new double[]{0,0,0};
        coords[getModeIndex()] = setpoint;
        drivetrain.setPosition(coords);
    }

    private int getModeIndex(){
        if(mode == PIDCalibration.CalibrationMode.X) return 0;
        if(mode == PIDCalibration.CalibrationMode.Y) return 1;
        if(mode == PIDCalibration.CalibrationMode.PHI) return 2;
        else return -1;
    }

    private static enum CalibrationMode { X,Y,PHI }

}
