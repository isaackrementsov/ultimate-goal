package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.ControlledDrivetrain;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;

public class PositionControlTest extends LinearOpMode {

    ControlledDrivetrain drivetrain;

    private double circumference = 10;
    private int ticksPerRev = 280;

    private double width = 30;
    private int backTicksPerDegree = 1;

    private double x0 = 0;
    private double y0 = 0;
    private double phi0 = 0;

    private final double TILE_SIZE = 60.95;

    public void runOpMode(){
        DcMotorX mRF= new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                mLF = new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB")),
                mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));


        DcMotorX wheelR = new DcMotorX(hardwareMap.dcMotor.get("wheelR"), ticksPerRev, circumference),
                wheelL = new DcMotorX(hardwareMap.dcMotor.get("wheelL"), ticksPerRev, circumference),
                wheelB = new DcMotorX(hardwareMap.dcMotor.get("wheelB"), ticksPerRev, circumference);

        Odometry positionTracker = new Odometry(wheelR, wheelL, wheelB, backTicksPerDegree, 10, width, x0, y0, phi0);
        drivetrain = new ControlledDrivetrain(mRF, mLF, mRB, mLB, positionTracker, true);

        Thread drivetrainThread = new Thread(drivetrain);

        waitForStart();
        drivetrainThread.start();

        drivetrain.setPosition(TILE_SIZE, TILE_SIZE*3, 20);
        drivetrain.setActive(true);

        while(!isStopRequested());
    }

}
