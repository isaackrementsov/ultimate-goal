package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.ControlledDrivetrain;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;

@Autonomous
public class PositionControlTest extends LinearOpMode {

    // Encoder wheel information
    private double circumference = 15.71;
    private int ticksPerRev = 8192;
    // Robot width and back encoder wheel ticks per degree
    private double width = 30;
    private double backDistancePerDegree = -41.577/(2*Math.PI);
    // Initial position
    private double x0 = 0;
    private double y0 = 0;
    private double phi0 = 0;
    // Size of a tile on the field (in cm)
    private final double TILE_SIZE = 60.95;

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
        Odometry positionTracker = new Odometry(wheelR, wheelL, wheelB, 50, backDistancePerDegree, width, x0, y0, phi0);

        // Instantiate the PID-controlled drivetrain
        drivetrain = new ControlledDrivetrain(mRF, mLF, mRB, mLB, positionTracker);
        drivetrain.reverse();
        drivetrain.telemetry = telemetry;
        // Run it in a separate thread
        Thread drivetrainThread = new Thread(drivetrain);

        waitForStart();

        // Start the thread
        drivetrainThread.start();
        // Tell the drivetrain to actively correct position errors
        drivetrain.setActive(true);

        // Drive to (x=1 tile, y=3 tiles, heading=20 degrees)
        drivetrain.setPosition(0, TILE_SIZE*3, 0);

        try {
            Thread.sleep(50);
        }catch(Exception e){ }

        while(!isStopRequested() && drivetrain.isBusy());

        drivetrain.setBrake(true);
        drivetrain.stop();
        drivetrain.setActive(false);
        drivetrain.stopController();
    }

}
