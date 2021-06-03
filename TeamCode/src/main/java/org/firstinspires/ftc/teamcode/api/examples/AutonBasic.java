package org.firstinspires.ftc.teamcode.api.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Drivetrain;

@Autonomous
public class AutonBasic extends LinearOpMode {

    private Drivetrain drivetrain;

    public void runOpMode(){
        DcMotorX mRF= new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                mLF = new DcMotorX(hardwareMap.dcMotor.get("mLF")),
                mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB")),
                mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));

        drivetrain = new Drivetrain(mRF, mLF, mRB, mLB);

        waitForStart();

        long start = System.currentTimeMillis();

        while(System.currentTimeMillis() - start < 1000 && !isStopRequested()){
            drivetrain.drive(0.5, Drivetrain.Direction.FORWARD);
        }

        drivetrain.stop();
    }

}
