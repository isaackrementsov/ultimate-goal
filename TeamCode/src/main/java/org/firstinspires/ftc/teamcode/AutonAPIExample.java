package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.api.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class AutonAPIExample extends LinearOpMode {

    private Robot bot;

    public void runOpMode() {
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDrivetrain(
            new String[]{"mRF", "mLF", "mRB", "mLB"},
            new double[]{32, 32, 32, 32},
            new double[]{560, 560, 560, 560},
            1,
            false
        );

        waitForStart();

        bot.drive(0.2, 30, Robot.Direction.FORWARD);
    }

}