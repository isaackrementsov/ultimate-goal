package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.api.DcMotorX;
import org.firstinspires.ftc.teamcode.api.Odometry;
import org.firstinspires.ftc.teamcode.api.Robot;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;

@TeleOp
public class OdometryTest extends OpMode {

    private Robot bot;
    private Odometry positionTracker;
    private double power = 0.5;

    private double circumference = 15.71;
    private int ticksPerRev = 8192;

    private double width = 40.8;
    private double backDistancePerRadian = 41.577/(2*Math.PI);

    private double x0 = 0;
    private double y0 = 0;
    private double phi0 = 0;

    private boolean measuring = false;
    private double lastBackDistance = 0;
    private double totalBackDistance = 0;
    private double nAvg = 0;

    private double theta0 = 0;
    private double backDistance0 = 0;
    private boolean measuringVariable = false;
    private ArrayList<Double> theta = new ArrayList<>();
    private ArrayList<Double> backDistance = new ArrayList<>();
    private Exception saveError = null;

    private Robot.ButtonState lastButtons1 = new Robot.ButtonState();

    @Override
    public void init() {
        this.bot = new Robot(hardwareMap, telemetry);
        bot.addDrivetrain(new String[]{"mRF", "mLF", "mRB", "mLB"}, true);

        DcMotorX motorR = new DcMotorX(hardwareMap.dcMotor.get("mRB"), ticksPerRev, circumference);
        DcMotorX motorL = new DcMotorX(hardwareMap.dcMotor.get("mLF"), ticksPerRev, circumference);
        DcMotorX motorB = new DcMotorX(hardwareMap.dcMotor.get("mRF"), ticksPerRev, circumference);

        positionTracker = new Odometry(motorR, motorL, motorB, 50, backDistancePerRadian, width, x0, y0, phi0);

        Thread positionThread = new Thread(positionTracker);
        positionThread.start();
    }

    @Override
    public void loop() {
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;
        double rightY = -gamepad1.right_stick_y; // Reads negative from the controller

        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean aHit = a && !lastButtons1.a;
        boolean xHit = x && !lastButtons1.x;
        boolean yHit = y && !lastButtons1.y;

        // Drive the robot with joysticks if they are moved
        if(Math.abs(leftX) > .1 || Math.abs(rightX) > .1 || Math.abs(rightY) > .1) {
            bot.drive(power, leftX, rightX, rightY);
        }else{
            // If the joysticks are not pressed, do not move the bot
            bot.stop();
        }

        // Keep average ticks/rotation
        if(aHit){
            // Stop measuring
            if(measuring){
                nAvg++;
                totalBackDistance += Math.abs(positionTracker.wheelB.getPosition() - lastBackDistance);
                measuring = false;
            }
            // Start measuring
            else{
                measuring = true;
                lastBackDistance = positionTracker.wheelB.getPosition();
            }
        }

        // Show running average
        if(b){
            telemetry.addData("Average back distance per revolution:", totalBackDistance/nAvg);
        }

        if(xHit){
            if(measuringVariable){
                measuringVariable = false;
            }else{
                backDistance0 = positionTracker.wheelB.getPosition();
                theta0 = positionTracker.phi;
                measuringVariable = true;
            }
        }

        if(yHit){
            saveError = saveCSV();
        }

        if(measuringVariable){
            Double deltaB = Math.abs(positionTracker.wheelB.getPosition() - backDistance0);
            Double deltaTheta = Math.abs(positionTracker.phi - theta0);

            backDistance.add(deltaB);
            theta.add(deltaTheta);
        }

        telemetry.addData("Measuring rotation", measuring);
        telemetry.addData("Measuring rotation function", measuringVariable);

        if(saveError != null) telemetry.addData("Save error", saveError);

        telemetry.addData("x", positionTracker.x);
        telemetry.addData("y", positionTracker.y);
        telemetry.addData("phi", positionTracker.phi*(180/Math.PI));
        telemetry.addData("Odometry cycling time", positionTracker.actualTime);

        lastButtons1.update(a, false, x, y);
    }

    private Exception saveCSV(){
        try {
            File file = AppUtil.getInstance().getSettingsFile("backdistance.csv");
            FileWriter csvWriter = new FileWriter(file);

            csvWriter.append("T");
            csvWriter.append(",");
            csvWriter.append("B");
            csvWriter.append("\n");

            for(int i = 0; i < theta.size(); i++){
                csvWriter.append(theta.get(i).toString());
                csvWriter.append(",");
                csvWriter.append(backDistance.get(i).toString());
                csvWriter.append("\n");
            }

            csvWriter.flush();
            csvWriter.close();

            telemetry.addData("Successfully saved csv", "");

            return null;
        }catch (Exception e){
            telemetry.addData("Error Saving File", e);

            return e;
        }
    }

    @Override
    public void stop(){
        positionTracker.stop();
    }
}
