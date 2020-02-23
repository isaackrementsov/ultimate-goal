package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.Robot;

public class AutonCompetition extends LinearOpMode {

    private Robot bot;
    private boolean team1 = true;

    public void runOpMode() {
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDrivetrain (
                new String[]{"mRF", "mLF", "mRB", "mLB"},
                new double[]{32, 32, 32, 32},
                new double[]{560, 560, 560, 560},
                1,
                false
        );

        waitForStart();

        double power = 0.2;

        if(team1) {
            bot.drive(power, 57, Robot.Direction.FORWARD);
            bot.drive(power, 114, Robot.Direction.LEFT);

            bot.rotate(power, 90);
            bot.drive(power, 228, Robot.Direction.FORWARD);

            bot.rotate(power, 180);
            bot.drive(power, 114, Robot.Direction.FORWARD);
        }else {
            bot.drive(power, 57, Robot.Direction.LEFT);
            bot.drive(power, 57, Robot.Direction.FORWARD);

            bot.rotate(power, 180);
            bot.drive(power, 114, Robot.Direction.FORWARD);
            bot.drive(power, 342, Robot.Direction.RIGHT);
            bot.drive(power, 114, Robot.Direction.FORWARD);

            bot.rotate(power, 180);
            bot.drive(power, 114, Robot.Direction.FORWARD);
            bot.drive(power, 171, Robot.Direction.RIGHT);
        }
    }

}
