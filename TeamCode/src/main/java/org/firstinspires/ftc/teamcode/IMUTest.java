package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.Robot;

public class IMUTest extends LinearOpMode {

    private Robot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDrivetrain(
                new String[]{"mRF", "mLF", "mRB", "mRF"},
                new double[]{31.42, 31.42, 31.42, 31.42},
                new double[]{767.2, 767.2, 767.2, 767.2},
                1,
                true
        );
        bot.configureIMU();

        waitForStart();

        while(!isStopRequested()){
            bot.getAngularOrientation();
        }
    }
}
