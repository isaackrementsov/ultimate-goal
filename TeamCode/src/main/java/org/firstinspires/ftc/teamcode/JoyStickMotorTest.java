package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.api.Robot;

public class JoyStickMotorTest extends OpMode {

    private Robot bot;

    public void init(){
        this.bot = new Robot(hardwareMap, telemetry);
        bot.addDcMotor("motor", true);
    }

    public void loop(){
        if(gamepad1.right_trigger > 0.1 ){
            bot.dcMotors.get("motor").setPower(gamepad1.right_trigger);
        }else if(gamepad1.left_trigger > 0.1){
            bot.dcMotors.get("motor").setPower(-gamepad1.left_trigger);
        }
    }

}
