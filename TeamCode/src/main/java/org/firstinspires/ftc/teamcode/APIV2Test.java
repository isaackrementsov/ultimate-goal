package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Drivetrain;

@TeleOp
public class APIV2Test extends OpMode {

    private Drivetrain drivetrain;
    private double power = 0.5;

    public void init(){
        DcMotorX mRF= new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                mLF = new DcMotorX(hardwareMap.dcMotor.get("mRF")),
                mRB = new DcMotorX(hardwareMap.dcMotor.get("mRB")),
                mLB = new DcMotorX(hardwareMap.dcMotor.get("mLB"));

        mRF.runWithoutEncoder();
        mLF.runWithoutEncoder();
        mRB.runWithoutEncoder();
        mLB.runWithoutEncoder();

        drivetrain = new Drivetrain(mRF, mLF, mRB, mLB);
    }

    public void loop(){
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y; // Reads negative from the controller

        // Drive the robot with joysticks if they are moved
        if(Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1) {
            telemetry.addData("Right y", rightY);
            telemetry.addData("Motor power", drivetrain.mRF.getPower());
            telemetry.addData("Motor 2 power", drivetrain.mLF.getPower());
            drivetrain.driveWithGamepad(power, rightY, rightX, leftX);
        }else{
            // If the joysticks are not pressed, do not move the bot
            drivetrain.stop();
        }
    }

}
