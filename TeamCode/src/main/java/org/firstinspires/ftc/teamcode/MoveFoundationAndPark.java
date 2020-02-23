// TODO: CREATE MORE OPMODES FOR EACH TEAM STARTING POS!!!
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.api.Robot;

public class MoveFoundationAndPark extends LinearOpMode {

    private Robot bot;

    private TouchSensor bumper1;
    private TouchSensor bumper2;

    private final double TILE_SIZE = 60.96;

    private final double Y_TO_FOUNDATION = 3.3*TILE_SIZE;

    private double LOW_POWER = 0.2;
    private double HIGH_POWER = 1;

    private Robot.Direction CENTER_DIRECTION = Robot.Direction.LEFT;

    @Override
    public void runOpMode() throws  InterruptedException {
        this.bot = new Robot(hardwareMap, telemetry);
        bot.addDrivetrain(
                new String[]{"mRF", "mLF", "mRB", "mLB"},
                new double[]{31.42, 31.42, 31.42, 31.42},
                new double[]{767.2, 767.2, 767.2, 767.2},
                1.54,
                true
        );

        bot.addServo("intakeFold", 180, 180, 0);
        bot.addServo("bumper", 180, 115, 60);

        bumper1 = hardwareMap.touchSensor.get("bumperLimit1");
        bumper2 = hardwareMap.touchSensor.get("bumperLimit2");

        bot.resetServo("bumper", 0);

        waitForStart();

        //driveToFoundation();
        moveFoundation();
        //park();
    }

    // TODO: Make sure this is the right distance
    private void park(){
        bot.drive(HIGH_POWER, TILE_SIZE*1.5, Robot.Direction.BACKWARD);
    }

    // TODO: Test distances here
    private void moveFoundation() throws InterruptedException {
        while(!(bumper1.isPressed() && bumper2.isPressed()) && opModeIsActive()){
            bot.drive(LOW_POWER, Robot.Direction.BACKWARD);
        }

        /*bot.drive(LOW_POWER, Robot.Direction.BACKWARD);

        Thread.sleep(500);*/

        bot.rotateServo("bumper", 60, 1000);

        bot.stop();
        /*bot.rotate(HIGH_POWER, 50);
        bot.drive(LOW_POWER, 10, Robot.Direction.LEFT);
        bot.rotate(HIGH_POWER, -30);

        bot.rotateServo("bumper", 65, 0);*/
    }

    private void driveToFoundation(){
        bot.drive(LOW_POWER, TILE_SIZE, CENTER_DIRECTION);

        bot.rotateServo("intakeFold", 0, 0);

        bot.drive(HIGH_POWER, Y_TO_FOUNDATION, Robot.Direction.FORWARD);
        bot.stop();

        bot.rotate(HIGH_POWER, 90.0);
    }

}
