package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.api.Robot;

@TeleOp
public class Drive extends OpMode {

    private Robot bot;
    private double power;

    private final String VUFORIA_KEY = "AY3aN3z/////AAABmUIe2Kd1wEt0nkr2MAal4OQiiEFWa3aLCHRnFBO1wd2HDT+GFXOTpcrhqEiZumOHpODdyVc55cYOiTSxpPrN+zfw7ZYB8X5z3gRLRIhPj4BJLD0/vPTKil7rDPSluUddISeCHL1HzPdIfiZiG/HQ89vhBdLfrWpngKLF4tH4FB4YWdKZu5J9EBtVTlXqR1OUXVTM3p9DepM9KukrVxMESF/ve+RYix7UXMO5qbljnc/LjQdplFO8oX4ztEe3aMXN14GadXggrfW+0m3nUmT8rXNTprc62LR1v0RbB4L+0QWfbgSDRyeMdBrvg8KIKLb1VFVrgUecbYBtHTTsLZALnU7oOOARnfGdtHC0aG3FAGxg";
    private final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private final String QUAD_LABEL = "Quad";
    private final String SINGLE_LABEL = "Single";

    private boolean readyToShoot = false;

    public void init(){
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDrivetrain(new String[]{"mRF", "mLF", "mRB", "mLB"}, true);

        bot.addDcMotor("intake", true);
        bot.addDcMotor("arm", 360, 288, true, true);
        bot.addDcMotor("intakeWheels", false);
        bot.addDcMotor("launcher", false);

        bot.addServo("flipper");

        power = 0.5;
    }

    private void loopGamepad1(){
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y; // Reads negative from the controller
        double triggerRight = gamepad1.right_trigger;
        double triggerLeft = gamepad1.left_trigger;
        boolean bumperRight = gamepad1.right_bumper;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;

        if (gamepad1.dpad_up) power = 0.9;
        if (gamepad1.dpad_right) power = 0.5;
        if (gamepad1.dpad_left) power = 0.3;
        if (gamepad1.dpad_down) power = 0.2;

        telemetry.addData("RightY", rightY);
        telemetry.addData("RightX", rightX);
        telemetry.addData("LeftX", leftX);

        // Drive the robot with joysticks if they are moved
        if (Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1) {
            bot.drive(power, leftX, rightX, rightY);
        } else {
            // If the joysticks are not pressed, do not move the bot
            bot.stop();
        }

        double intakePower = 0;
        if(Math.abs(triggerRight) > 0.05) {
            intakePower = triggerRight;
        }else if(Math.abs(triggerLeft) > 0.05){
            intakePower = -triggerLeft;
        }

        bot.moveDcMotor("intake", 2*intakePower);

        // Power the Wobble Goal Arm
        // TODO: Make this setVelocity not setPower!
        double armPower = 0;
        if(a){
            armPower = 0.5;
        }else if(b){
            armPower = -0.3;
        }

        // Move the arm between + or - 40 degrees
        bot.moveDcMotor("arm", 180, armPower, true);

        if(bumperRight){
            if(readyToShoot){
                bot.rotateServo("flipper", 180, 0);
            }else{
                bot.rotateServo("flipper", 120, 0);
            }

            readyToShoot = !readyToShoot;
        }

        // Turn intake wheels on/off
        if(x){
            if(bot.getMotorPower("intakeWheels") == 0){
                bot.moveDcMotor("intakeWheels", 1);
            }else{
                bot.moveDcMotor("intakeWheels", 0);
            }
        }

        // Turn launcher flywheel on/off
        if(y){
            if(bot.getMotorPower("launcher") == 0){
                bot.moveDcMotor("launcher", 1);
            }else{
                bot.moveDcMotor("launcher", 0);
            }
        }
    }

    @Override
    public void loop(){
        loopGamepad1();
    }

    @Override
    public void stop() {
        // Shutdown TensorFlow
        bot.shutDownCV();
        super.stop();
    }
}
