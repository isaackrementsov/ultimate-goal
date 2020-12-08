package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.DcMotorX;

@Autonomous
public class OpenCVTest extends LinearOpMode {

    private DcMotorX intake;

    public void runOpMode(){
        intake = new DcMotorX(hardwareMap.dcMotor.get("intakeWheels"));

        waitForStart();

        intake.core.setPower(1);

        try {
            Thread.sleep(3000);
        }catch(Exception e){

        }
    }

}
