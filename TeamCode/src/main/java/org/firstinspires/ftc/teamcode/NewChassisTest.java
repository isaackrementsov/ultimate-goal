package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.Robot;

public class NewChassisTest extends LinearOpMode {
    private Robot bot;

    public void runOpMode() {
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDcMotor("drivetrainLeft", false);
        bot.addDcMotor("drivetrainRight", false);

        waitForStart();

        while(!isStopRequested()){
            bot.dcMotors.get("drivetrainRight").setPower(1);
            bot.dcMotors.get("drivetrainLeft").setPower(1);
        }
    }
}
