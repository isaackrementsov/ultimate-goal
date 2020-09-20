package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.api.Robot;

@TeleOp
public class MiniBotServoTest extends OpMode {

    private Robot bot;

    public void init(){
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addServo("testServo", 180, 180, 0, 0);
        bot.resetServo("testServo", 1000);
    }

    public void loop(){
        if(gamepad1.dpad_up){
            bot.incrementServo("testServo", 1, 0);
        }
        if(gamepad1.dpad_down){
            bot.incrementServo("testServo", -1, 0);
        }
    }

}
