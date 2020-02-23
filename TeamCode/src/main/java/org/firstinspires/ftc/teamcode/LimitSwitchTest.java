package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.api.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class LimitSwitchTest extends LinearOpMode {

    private Robot bot;

    public void runOpMode() {
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addLimitedMotor("mRF", "limitLower", "limitUpper", 32, 560, true);

        waitForStart();

        bot.moveLimitedMotor("mRF", 200, 0.2);
        bot.moveLimitedMotor("mRF", 200, -0.2);
    }

}