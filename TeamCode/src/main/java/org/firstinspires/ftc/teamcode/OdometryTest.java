package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.firstinspires.ftc.teamcode.api.Robot;

public class OdometryTest extends OpMode {

    private Robot bot;
    private Odometry positionTracker;
    private double power = 0.5;

    private double circumference = 10;
    private int ticksPerRev = 280;

    private double width = 30;
    private int backTicksPerDegree = 1;

    private double x0 = 0;
    private double y0 = 0;
    private double phi0 = 0;

    @Override
    public void init() {
        this.bot = new Robot(hardwareMap, telemetry);
        bot.addDrivetrain(new String[]{"mRF", "mLF", "mRB", "mLB"}, true);

        DcMotorX motorR = new DcMotorX(hardwareMap.dcMotor.get("motorR"), ticksPerRev, circumference);
        DcMotorX motorL = new DcMotorX(hardwareMap.dcMotor.get("motorL"), ticksPerRev, circumference);
        DcMotorX motorB = new DcMotorX(hardwareMap.dcMotor.get("motorB"), ticksPerRev, circumference);

        positionTracker = new Odometry(motorR, motorL, motorB, backTicksPerDegree, 10, width, x0, y0, phi0);

        Thread positionThread = new Thread(positionTracker);
        positionThread.start();
    }

    @Override
    public void loop() {
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y; // Reads negative from the controller

        // Drive the robot with joysticks if they are moved
        if(Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1) {
            bot.drive(power, leftX, rightX, rightY);
        }else{
            // If the joysticks are not pressed, do not move the bot
            bot.stop();
        }

        telemetry.addData("x", positionTracker.x);
        telemetry.addData("y", positionTracker.y);
        telemetry.addData("phi", positionTracker.phi);
    }

    @Override
    public void stop(){
        positionTracker.stop();
    }
}
