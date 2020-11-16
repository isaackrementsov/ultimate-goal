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

    public void init(){
        this.bot = new Robot(hardwareMap, telemetry);

        bot.addDrivetrain(new String[]{"mRF", "mLF", "mRB", "mLB"}, true);
        bot.addDcMotor("intake", true);
        bot.addDcMotor("arm", 360, 288, true, true);

        bot.addServo("flipper");

        /*bot.initCV(
            VUFORIA_KEY,
            VuforiaLocalizer.CameraDirection.BACK,
            TFOD_MODEL_ASSET,
            new String[]{QUAD_LABEL, SINGLE_LABEL}
        );*/

        power = 0.5;
    }

    @Override
    public void loop(){
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y; // Reads negative from the controller
        double triggerRight = gamepad1.right_trigger;
        double triggerLeft = gamepad1.left_trigger;
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

        // Auto-align when x is pressed
        // TODO: Make auto-align method for TeleOp by removing loops
        /*if(x){
            try {
                bot.autoAlign(SINGLE_LABEL, 0.5);
            }catch(Robot.CVInitializationException e){
                telemetry.addData("TensorFlow Error:", e);
            }
        }*/

        double armPower = 0;
        if(a){
            armPower = 0.5;
        }else if(b){
            armPower = -0.3;
        }

        // Move the arm between + or - 40 degrees
        //bot.moveWithEncodedLimit("arm", -40, 40, armPower);
        bot.moveDcMotor("arm", armPower);

        if(y){
            double position = bot.getServoPos("flipper");

            if(position >= 59){
                bot.rotateServo("flipper", 60, 0);
            }else{
                bot.rotateServo("flipper", 0, 0);
            }
        }
    }

    @Override
    public void stop() {
        // Shutdown TensorFlow
        bot.shutDownCV();
        super.stop();
    }
}
