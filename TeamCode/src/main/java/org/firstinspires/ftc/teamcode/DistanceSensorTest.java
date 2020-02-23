package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.Robot;

public class DistanceSensorTest extends LinearOpMode {

    private Robot bot;

    public void runOpMode() {
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDistanceSensor("color", false);

        while(!isStopRequested()){
            double distance = bot.getDistanceCM("color");
            telemetry.addData("Current Distance: ", distance);
            telemetry.update();
        }
    }
}
