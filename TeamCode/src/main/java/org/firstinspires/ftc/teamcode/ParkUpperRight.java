package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.Robot;

@Autonomous
public class ParkUpperRight extends LinearOpMode {

    private Robot bot;

    private double TILE_SIZE = 60.96;

    @Override
    public void runOpMode() throws InterruptedException {

        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDrivetrain(
                new String[]{"mRF", "mLF", "mRB", "mLB"},
                new double[]{31.42, 31.42, 31.42, 31.42},
                new double[]{767.2, 767.2, 767.2, 767.2},
                1,
                true
        );
        waitForStart();

        bot.drive(1, 0.5*TILE_SIZE, Robot.Direction.FORWARD);

        bot.stop();
    }
}
