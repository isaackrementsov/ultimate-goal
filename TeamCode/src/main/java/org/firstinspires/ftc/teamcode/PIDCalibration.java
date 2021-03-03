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
    private final double TILE_SIZE = 59.69;

    // Calibration parameters
    private double Kp = 3.01;
    private double Ki = 2e-17;
    private double Kd = 0.304;

    // For editing coeffients in the OpMode
    private int multiplier = 0;
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
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y; // Reads negative from the controller

        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadRight = gamepad1.dpad_right;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;

        boolean aHit = a && !lastButtons.a;
        boolean bHit = b && !lastButtons.b;
        boolean xHit = x && !lastButtons.x;
        boolean dpadUpHit = dpadUp && !lastDpads.dpad_up;
        boolean dpadDownHit = dpadDown && !lastDpads.dpad_down;
        boolean dpadRightHit = dpadRight && !lastDpads.dpad_right;
        boolean dpadLeftHit = dpadLeft && !lastDpads.dpad_left;

        // Stop position control
        if(bHit){
            drivetrain.setActive(false);
            drivetrain.stop();
        }

        if(drivetrain.getActive()) {
            telemetry.addData("Setpoint", setpoint);
            telemetry.addData("Mode index", getModeIndex());
            telemetry.addData("x", drivetrain.positionTracker.x);
            telemetry.addData("y", drivetrain.positionTracker.y);
            telemetry.addData("Heading", drivetrain.positionTracker.phi);
        }else{
            // Start/restart position control
            if(aHit){
                drivetrain.positionTracker.reset();
                drivetrain.stop();

                setPosition();
                drivetrain.setActive(true);
            }

            // Select a coefficient to edit
            if(xHit){
                editingIdx = editingIdx < editingOptions.length - 1 ? editingIdx + 1 : 0;
            }

            // Increment selected coefficient
            if(dpadUpHit) editCoefficients(Math.pow(10, multiplier));
            else if(dpadDownHit) editCoefficients(-Math.pow(10, multiplier));

            // Multiply increments by 10^multiplier for fine/coarse tuning
            if(dpadRightHit) multiplier++;
            else if(dpadLeftHit) multiplier--;

            // Allow for manual driving
            if(Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1){
                drivetrain.driveWithGamepad(0.5, rightY, leftX, rightX);
            }else{
                drivetrain.stop();
            }

            // Log current coefficient values
            telemetry.addData("Currently editing", editingOptions[editingIdx]);
            telemetry.addData("Increment size", Math.pow(10, multiplier));
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
        }

        lastDpads.update(dpadUp, dpadDown, dpadRight, dpadLeft);
        lastButtons.update(a, b, x, false);
    }

    public void stop(){
        drivetrain.setActive(false);
        drivetrain.stop();
        drivetrain.stopController();
    }

    private void initializeCoefficients(){
        int i = getModeIndex();

        drivetrain.Kp[i] = Kp;
        drivetrain.Kd[i] = Kd;
        drivetrain.Ki[i] = Ki;
    }

    private void editCoefficients(double change){
        if(editingIdx == 0) Kp += change;
        if(editingIdx == 1) Kd += change;
        if(editingIdx == 2) Ki += change;

        initializeCoefficients();
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
