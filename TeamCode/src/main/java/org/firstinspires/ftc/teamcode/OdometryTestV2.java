package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Drivetrain;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.firstinspires.ftc.teamcode.api.Robot;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;

@TeleOp
public class OdometryTestV2 extends OpMode {

    private Drivetrain drivetrain;
    private Odometry positionTracker;
    private double power = 0.5;

    private double circumference = 15.71;
    private int ticksPerRev = 8192;

    private double width = 40.8;
    private double backDistancePerRadian = -41.577/(2*Math.PI);

    private double x0 = 0;
    private double y0 = 0;
    private double phi0 = 0;

    @Override
    public void init() {
        DcMotorX mRF = new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF")),
                mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB")),
                mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));

        drivetrain = new Drivetrain(mRF, mLF, mRB, mLB);
        drivetrain.setBrake(true);
        drivetrain.reverse();

        DcMotorX motorR = new DcMotorX(hardwareMap.dcMotor.get("mRB"), ticksPerRev, circumference);
        DcMotorX motorL = new DcMotorX(hardwareMap.dcMotor.get("mLF"), ticksPerRev, circumference);
        DcMotorX motorB = new DcMotorX(hardwareMap.dcMotor.get("mRF"), ticksPerRev, circumference);

        positionTracker = new Odometry(motorR, motorL, motorB, 50, backDistancePerRadian, width, x0, y0, phi0);

        Thread positionThread = new Thread(positionTracker);
        positionThread.start();
    }

    @Override
    public void loop() {
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y; // Reads negative from the controller

        if(Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1){
            drivetrain.driveWithGamepad(0.5, rightY, leftX, rightX);
        }else{
            drivetrain.stop();
        }

        telemetry.addData("x", positionTracker.x);
        telemetry.addData("y", positionTracker.y);
        telemetry.addData("phi", positionTracker.phi*180/Math.PI);
    }

    @Override
    public void stop(){
        positionTracker.stop();
    }
}
