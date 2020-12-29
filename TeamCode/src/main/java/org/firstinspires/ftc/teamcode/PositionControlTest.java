package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.ControlledDrivetrain;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;

@Autonomous
public class PositionControlTest extends LinearOpMode {

    // Encoder wheel information
    private double circumference = 10;
    private int ticksPerRev = 280;
    // Robot width and back encoder wheel ticks per degree
    private double width = 30;
    private double backDistancePerDegree = 0.5;
    // Initial position
    private double x0 = 0;
    private double y0 = 0;
    private double phi0 = 0;
    // Size of a tile on the field (in cm)
    private final double TILE_SIZE = 60.95;

    public void runOpMode(){
        // Get all of the drivetrain motors
        DcMotorX mRF= new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                mLF = new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB")),
                mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));
        // Get the odometry wheels
        DcMotorX wheelR = new DcMotorX(hardwareMap.dcMotor.get("wheelR"), ticksPerRev, circumference),
                wheelL = new DcMotorX(hardwareMap.dcMotor.get("wheelL"), ticksPerRev, circumference),
                wheelB = new DcMotorX(hardwareMap.dcMotor.get("wheelB"), ticksPerRev, circumference);

        // Create an odometry instance for the drivetrain
        Odometry positionTracker = new Odometry(wheelR, wheelL, wheelB, 10, backDistancePerDegree, width, x0, y0, phi0);
        // Instantiate the PID-controlled drivetrain
        ControlledDrivetrain drivetrain = new ControlledDrivetrain(mRF, mLF, mRB, mLB, positionTracker);
        // Run it in a separate thread
        Thread drivetrainThread = new Thread(drivetrain);

        waitForStart();
        // Start the thread
        drivetrainThread.start();
        // Tell the drivetrain to actively correct position errors
        drivetrain.setActive(true);

        // Drive to (x=1 tile, y=3 tiles, heading=20 degrees)
        drivetrain.setPosition(TILE_SIZE, TILE_SIZE*3, 20);
        // Wait for the robot to get to the target position
        while(!isStopRequested() && drivetrain.isBusy());
        // Go back to the original position
        drivetrain.setPosition(x0, y0, phi0);
        // Wait for the robot to get back
        while(!isStopRequested() && drivetrain.isBusy());
    }

}
