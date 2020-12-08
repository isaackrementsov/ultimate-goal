/*

Code written 2019-2020,
By:
    * Isaac Krementsov
    * Kaiden Cook
 - CVU Robohawks 5741 -

The purpose of the Robot class is to provide a universal and
reusable API for programming the robot. Instead of muddling around
with FTC's poor programming documentation and unclear features, this
API provides clear documentation and succinct OpMode code.

Check out the documentation here:
https://www.notion.so/RoboHawks-Documentation-7f03a74bd3c747f7b37bca76e656741b

*/

package org.firstinspires.ftc.teamcode.api;

import android.sax.StartElementListener;
import android.text.method.Touch;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Robot {

    // The hardwareMap is used to get motors, as they cannot be directly stored as references
    private HardwareMap hardwareMap;
    // Telemetry is only here for debugging; should be removed before release
    private Telemetry telemetry;

    // These are the *4* motors that will be used for driving - they must be stored as [right-front, left-front, right-back, left-back]
    // The wheels must be mecanum and aligned properly
    private String[] drivetrain = new String[4];

    // Here are the actual DcMotor references, for the drivetrain and other motors; indexed by HardwareMap name
    public HashMap<String, DcMotor> dcMotors = new HashMap<>();
    // This stores important info for motors: circumference and encoder ticks per revolution
    public HashMap<String, Double[]> dcMotorInfo = new HashMap<>();

    // Here, we store servo references; also indexed by HardwareMap name
    public HashMap<String, Servo> servos = new HashMap<>();
    // For incrementing a servo's position, we keep track of its current position here
    private HashMap<String, Double> servoPositions = new HashMap<>();
    // This stores the total rotation angle, minimum angle, and maximum angle for a servo
    private HashMap<String, Double[]> servoLimits = new HashMap<>();

    // These are references for motor limit sensors, indexed by HardwareMap name
    // They are stores as back sensor, front sensor
    public HashMap<String, TouchSensor[]> limitSwitches = new HashMap<>();
    // Some motors require more than 2 limits, so this stores an array of back sensors and an array of front sensors
    public HashMap<String, TouchSensor[][]> limitSwitchArrays = new HashMap<>();
    // This keeps track of whether or not limited motors should float when power is set to 0 - they need to brake when they hit limits
    private HashMap<String, Boolean> limitedMotorInfo = new HashMap<>();

    // Here are the distance sensor references (can include Color-Distance sensors); indexed by HardwareMap name
    public HashMap<String, DistanceSensor> distanceSensors = new HashMap<>();

    // Here are the optical or non-optical color sensor references; indexed by HardwareMap name
    public HashMap<String, ColorSensor> colorSensors = new HashMap<>();
    // This stores the scaleFactor of each color sensor, which can adjust downtowned colors
    private HashMap<String, Integer> colorSensorInfo = new HashMap<>();

    // Stores actions for event triggers
    private HashMap<String, Action> actions = new HashMap<>();

    // Built in gyro sensor for Rev hub
    public BNO055IMU imu;

    // This is the experimentally determined rotation coefficient of the bot (https://www.notion.so/Using-the-Robohawks-Team-API-0fb3006b5b564965b06749fd3fcd6c67#421a6b7240b743fd8aa075dbd2c60bfb)
    public double rotationCoefficient;

    // Computer vision information
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    // Have Vuforia and TensorFlow been initialized yet?
    public boolean isCVReady = false;

    public void configureIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void startIMU(){
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public double getAngularOrientation(){
        if(imu != null){
            Orientation orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES);

            return orientation.secondAngle;
        }else{
            return 0;
        }
    }

    // This adds a set of motors will full information for each motor
    public void addDrivetrain(String[] motors, double[] circumferences, double[] encoderTicks, double rotationalCoefficient, boolean reverseLeft){
        // Assign array of motor names to the drivetrain array
        drivetrain = motors;
        rotationCoefficient = rotationalCoefficient;

        // Add the set of motors and their info as DcMotor references; they all will need to brake
        addDcMotors(motors, circumferences, encoderTicks, true);

        for(int i = 0; i < drivetrain.length; i++){
            String motor = drivetrain[i];
            // Get the stored DcMotor reference
            DcMotor dcMotor = dcMotors.get(motor);

            // If index if even, this is a left motor
            if(i % 2 == 0 && reverseLeft){
                // If the left motors are put on backwards (they usually are), reverse them so that they go forward with positive power
                dcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            // Reset the current encoder reading
            dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Until the motors are triggered, the encoder will passively keep track of wheel rotation
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // In controller code, you usually don't need all of the encoder info
    public void addDrivetrain(String[] motors, boolean reverseLeft){
        addDrivetrain(motors, new double[]{1,1,1,1}, new double[]{0,0,0,0}, 1, reverseLeft);
    }

    // Set every motor to work in reverse (makes driving backwards easier if necessary)
    public void reverseDrivetrain(){
        for(String motor : drivetrain){
            DcMotor dcMotor = dcMotors.get(motor);
            DcMotorSimple.Direction direction = dcMotor.getDirection();
            DcMotorSimple.Direction newDirection = DcMotorSimple.Direction.REVERSE;

            if(direction.equals(DcMotorSimple.Direction.REVERSE)){
                newDirection = DcMotorSimple.Direction.FORWARD;
            }

            dcMotor.setDirection(newDirection);
        }
    }

    // Use the encoders to drive the bot a set distance in a set direction
    public void drive(double power, double distanceCM, Direction direction){
        double[] powers = new double[4];

        for(int i = 0; i < drivetrain.length; i++){
            String motor = drivetrain[i];
            double motorPower = power;

            switch(direction){
                // If you are moving the bot backwards, all motor powers should be negative
                case BACKWARD:
                    motorPower = -power;
                    break;

                // If you are moving the robot right, you are strafing
                case RIGHT:
                    // Strafing right means putting the right front and left back wheels in reverse
                    if(i == 0 || i == 3){
                        motorPower = -power;
                    }
                    break;

                // If you are moving the robot left, you are strafing in reverse
                case LEFT:
                    // Strafing left means putting the left front and right back wheels in reverse
                    if (i == 1 || i == 2){
                        motorPower = -power;
                    }
                    break;
            }
            // Otherwise, you are going forward, so keep the powers of each wheel the same

            Double[] info = dcMotorInfo.get(motor);

            double circumference = info[0];
            double encoderTicks = info[1];

            /*
                The target position is how many encoder ticks to rotate, which is the
                fraction of a rotation to move each wheel (target distance / wheel circumference),
                multiplied by ticks per rotation
            */
            int target = (int) (encoderTicks * distanceCM / circumference);
            // Either increase or decrease the current position by the target
            dcMotors.get(motor).setTargetPosition((motorPower < 0 ? -1 : 1) * target + dcMotors.get(motor).getCurrentPosition());
            // Set the motors to the FTC automatic encoder mode, RUN_TO_POSITION
            dcMotors.get(motor).setMode(DcMotor.RunMode.RUN_TO_POSITION);

            powers[i] = motorPower;
        }

        for(int i = 0; i < drivetrain.length; i++){
            // Power the motors in a separate loop to reduce delay between motors; this starts their movement
            dcMotors.get(drivetrain[i]).setPower(powers[i]);
        }

        // Wait for the motors to be done rotating before running any other code (all motors should stop at about the same time_
        while(dcMotors.get(drivetrain[0]).isBusy());
    }

    public void driveWithIMU(double power, double distanceCM, Direction direction){
        double[] powers = new double[4];

        for(int i = 0; i < drivetrain.length; i++){
            String motor = drivetrain[i];
            double motorPower = power;

            switch(direction){
                case BACKWARD:
                    motorPower = -power;
                    break;

                case RIGHT:
                    if(i == 0 || i == 3){
                        motorPower = -power;
                    }
                    break;

                case LEFT:
                    if (i == 1 || i == 2){
                        motorPower = -power;
                    }
                    break;
            }

            Double[] info = dcMotorInfo.get(motor);
            double circumference = info[0];
            double encoderTicks = info[1];
            int target = (int) (encoderTicks * distanceCM / circumference);

            dcMotors.get(motor).setTargetPosition((motorPower < 0 ? -1 : 1) * target + dcMotors.get(motor).getCurrentPosition());

            powers[i] = motorPower;
        }

        for(int i = 0; i < drivetrain.length; i++){
            dcMotors.get(drivetrain[i]).setPower(powers[i]);
        }

        DcMotor test = dcMotors.get(drivetrain[0]);

        while(Math.abs(test.getTargetPosition() - test.getCurrentPosition()) > 10);

        stop();
    }

    // Drive at a power, but not for a set distance
    public void drive(double power, Direction direction){
        for(int i = 0; i < drivetrain.length; i++){
            String motor = drivetrain[i];
            double motorPower = power;

            // Same power rules apply as in the distance drive method
            switch(direction) {
                case BACKWARD:
                    motorPower = -power;
                    break;

                case RIGHT:
                    if (i == 0 || i == 3) {
                        motorPower = -power;
                    }
                    break;

                case LEFT:
                    if (i == 1 || i == 2) {
                        motorPower = -power;
                    }
            }

            /*
                Instead of actually setting encoders, just power the motors.
                A second loop is not needed because to memory interactions or complex calculations
                are here to cause delay
            */
            dcMotors.get(motor).setPower(motorPower);
        }
    }

    // This is the controller-based driving method - it rotates (yaws) and translates the robot
    public void drive(double speed, double yaw, double strafe, double power){
        for(int i = 0; i < drivetrain.length; i++){
            double motorPower = 0;
            String motor = drivetrain[i];

            // Set a power based on motor position
            switch(i){
                case 0:
                    // For motor right front, +power=forward, +strafe=forward, +yaw=forward
                    motorPower = power - strafe - yaw;
                    break;
                case 1:
                    // For motor left front, +power=forward, +strafe=reverse, +yaw=reverse
                    motorPower = power + strafe + yaw;
                    break;
                case 2:
                    // For motor right back, +power=forward, +strafe=reverse, +yaw=forward
                    motorPower = power + strafe - yaw;
                    break;
                case 3:
                    // For motor left back, +power=forward, +strafe=forward, +yaw=reverse
                    motorPower = power - strafe + yaw;
            }

            // Mutliply power by the speed setting for the motor
            motorPower *= speed;

            // Set the power to move the motor indefinetly
            dcMotors.get(motor).setPower(motorPower);
        }
    }

    // Rotate the bot using encoders and the rotation coefficient
    public void rotate(double power, double angle){
        double[] powers = new double[4];

        for(int i = 0; i < drivetrain.length; i++){
            double motorPower = -power;

            // Both left motors will need to be reversed
            if(i % 2 == 1) {
                motorPower = power;
            }

            powers[i] = motorPower;

            String motor = drivetrain[i];
            Double[] info = dcMotorInfo.get(motor);

            double circumference = info[0];
            double encoderTicks = info[1];

            /*
                Set the target in the same way as driving, but use angle / rotationCoefficient
                as the target distance (see https://www.notion.so/Using-the-Robohawks-Team-API-0fb3006b5b564965b06749fd3fcd6c67#421a6b7240b743fd8aa075dbd2c60bfb)
            */
            int target = (int) (angle / rotationCoefficient * encoderTicks / circumference);
            dcMotors.get(motor).setTargetPosition((motorPower < 0 ? -1 : 1) * target + dcMotors.get(motor).getCurrentPosition());
        }

        // Set power for all of the motors
        for(int i = 0; i < drivetrain.length; i++){
            dcMotors.get(drivetrain[i]).setPower(powers[i]);
        }

        // Select a test motor to keep track of the drivetrain's overall progress in reaching the target position
        DcMotor test = dcMotors.get(drivetrain[0]);

        // Wait for the motors to get within 10 ticks of the target position
        while(Math.abs(test.getTargetPosition() - test.getCurrentPosition()) > 10);
        // Stop the robot once the target is reached
        stop();
    }

    public void rotateWithIMU(double power, double angle){
        double[] powers = new double[4];

        for(int i = 0; i < drivetrain.length; i++){
            double motorPower = power;

            if(i % 2 == 1){
                motorPower = -power;
            }

            powers[i] = motorPower;

            String motor = drivetrain[i];
            Double[] info = dcMotorInfo.get(motor);

            double circumference = info[0];
            double encoderTicks = info[1];

        }

        for(int i = 0; i < drivetrain.length; i++){
            dcMotors.get(drivetrain[i]).setPower(powers[i]);
        }

        double startAngle = getAngularOrientation();
        while(Math.abs(startAngle - getAngularOrientation()) > 2);

        stop();
    }

    // Set power to 0 for all of the drivetrain motors, which will brake the bot
    public void stop(){
        drive(0, 0, 0, 0);
    }

    public void addLimitTrigger(String[] switches, Action action, String actionId){ //TODO add and or behavior
        TouchSensor[] sensors = new TouchSensor[switches.length];

        for(int i = 0; i < switches.length; i++){
            sensors[i] = hardwareMap.touchSensor.get(switches[i]);
        }

        limitSwitches.put(actionId, sensors);
        actions.put(actionId, action);
    }

    public void driveUntilEventTriggered(double power, Direction direction, String actionId, LimitBehavior behavior){
        TouchSensor[] sensors = limitSwitches.get(actionId);

        while(!arePressed(sensors, behavior)){
            drive(power, direction);
        }

        actions.get(actionId).run(this);
    }

    // Add a simple DcMotor with no extra info
    public void addDcMotor(String motor, boolean brake){ addDcMotor(motor, 0, 0, brake); }

    public void addDcMotor(String motor, double circumference, double encoderTicks, boolean brake){
        addDcMotor(motor, circumference, encoderTicks, brake, false);
    }

    // Add a DcMotor with the info needed to use encoder positioning
    // If this is not a wheel, circumference is the distance it moves some part of the robot per revolution
    public void addDcMotor(String motor, double circumference, double encoderTicks, boolean brake, boolean reset){
        dcMotors.put(motor, hardwareMap.dcMotor.get(motor));
        dcMotorInfo.put(motor,  new Double[]{circumference, encoderTicks});

        if(brake) dcMotors.get(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(reset){
            dcMotors.get(motor).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotors.get(motor).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // Batch add DcMotors for ease of use
    public void addDcMotors(String motors[], double[] circumferences, double[] encoderTicks, boolean brake){
        for(int i = 0; i < motors.length; i++){
            addDcMotor(motors[i], circumferences[i], encoderTicks[i], brake);
        }
    }

    private int getEncoderValue(String motor, double distance){
        Double[] info = dcMotorInfo.get(motor);

        double circumference = info[0];
        double encoderTicks = info[1];

        return (int) (encoderTicks * distance / circumference);
    }

    private double encoderTicksToPosition(double positionTicks, double circumference, double encoderTicks){
        return circumference * positionTicks / encoderTicks;
    }

    public double getMotorPosition(String motor){
        Double[] info = dcMotorInfo.get(motor);

        double circumference = info[0];
        double encoderTicks = info[1];

        return encoderTicksToPosition(dcMotors.get(motor).getCurrentPosition(), circumference, encoderTicks);
    }

    public double getTargetPosition(String motor){
        Double[] info = dcMotorInfo.get(motor);

        double circumference = info[0];
        double encoderTicks = info[1];

        return encoderTicksToPosition(dcMotors.get(motor).getTargetPosition(), circumference, encoderTicks);
    }

    public double getMotorPower(String motor){
        return dcMotors.get(motor).getPower();
    }

    // Move a DcMotor indefinitely with a set power
    public void moveDcMotor(String motor, double motorPower){
        DcMotor motorToMove = dcMotors.get(motor);

        motorToMove.setPower(motorPower);
    }

    // Move a DcMotor for a set distance, similar to the distance drive method
    public void moveDcMotor(String motor, double distanceCM, double motorPower, boolean teleOp){
        Double[] info = dcMotorInfo.get(motor);

        // The encoder code used here is the same as what is used in the distance drive method
        int target = getEncoderValue(motor, distanceCM);
        dcMotors.get(motor).setTargetPosition((motorPower < 0 ? -1 : 1) * target + dcMotors.get(motor).getCurrentPosition());
        dcMotors.get(motor).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcMotors.get(motor).setPower(motorPower);

        // This code should be blocking in a LinearOpMode
        if(!teleOp){
            while(dcMotors.get(motor).isBusy());
        }
    }

    public double getControlledSpeed(double maxSpeed, double threshold, double error){
        double speed = Math.signum(error)*maxSpeed;

        if(Math.abs(error) < 2*threshold){
            speed *= error / (2*threshold);
        }

        return speed;
    }

    // Simplify using the existing moveDcMotor method
    public void moveToStaticPosition(String motor, double distanceCM, double motorPower, boolean teleOp){
        Double[] info = dcMotorInfo.get(motor);

        // The encoder code used here is the same as what is used in the distance drive method
        int target = getEncoderValue(motor, distanceCM);
        dcMotors.get(motor).setTargetPosition(target);
        dcMotors.get(motor).setPower(motorPower);

        if(!teleOp){
            double error = getTargetPosition(motor) - getMotorPosition(motor);
            double threshold = 1;

            while(Math.abs(error) > threshold){
                dcMotors.get(motor).setPower(getControlledSpeed(motorPower, threshold, error));
                error = getTargetPosition(motor) - getMotorPosition(motor);
            }

            dcMotors.get(motor).setPower(0);
        }
    }

    public void runAtConstantVelocity(String motor){
        dcMotors.get(motor).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Move between two encoder positions
    public void moveWithEncodedLimit(String motor, double limitLower, double limitUpper, double motorPower){
        double currentPosition = getMotorPosition(motor);
        telemetry.addData("Current position:", currentPosition);
        telemetry.addData("Can move?", currentPosition > limitLower && currentPosition < limitUpper);

        if(currentPosition > limitLower && currentPosition < limitUpper){
            moveDcMotor(motor, motorPower);
        }else{
            moveDcMotor(motor, 0);
        }
    }

    // Add a motor to be stopped with limit switches on either end of its range of motion
    public void addLimitedMotor(String motor, String limitLower, String limitUpper, boolean brake){
        addLimitedMotor(motor, limitLower, limitUpper, 0, 0, brake);
    }

    // Add a limited motor with the information necessary for encoders
    public void addLimitedMotor(String motor, String limitLower, String limitUpper, double circumference, double encoderTicks, boolean brake){
        // The main motor will be added as a normal DcMotor
        addDcMotor(motor, circumference, encoderTicks, brake);

        // Get the upper and lower switches (upper is hit by positive power, lower is hit by negative power)
        TouchSensor sensorLower = hardwareMap.touchSensor.get(limitLower);
        TouchSensor sensorUpper = hardwareMap.touchSensor.get(limitUpper);

        // Store the limit switch references and whether to brake the motor
        limitSwitches.put(motor, new TouchSensor[]{sensorLower, sensorUpper});
        limitedMotorInfo.put(motor, brake);
    }

    // Add a limited motor with multiple upper/lower limit switches
    public void addLimitedMotor(String motor, String[] limitsLower, String[] limitsUpper, boolean brake){
        addLimitedMotor(motor, limitsLower, limitsUpper, 0, 0, brake);
    }

    // Add the multiple limited motor with encoder information
    public void addLimitedMotor(String motor, String[] limitsLower, String[] limitsUpper, double circumference, double encoderTicks, boolean brake){
        addDcMotor(motor, circumference, encoderTicks, brake);

        // Loop though the limitsLower array and get the TouchSensor references from the hardwareMap
        TouchSensor[] sensorsLower = new TouchSensor[limitsLower.length];

        for(int i = 0; i < limitsLower.length; i++){
            sensorsLower[i] = hardwareMap.touchSensor.get(limitsLower[i]);
        }

        // Loop through the limitsUpper array and get the TouchSensor references from the hardwareMap
        TouchSensor[] sensorsUpper = new TouchSensor[limitsUpper.length];

        for(int i = 0; i < limitsUpper.length; i++){
            sensorsUpper[i] = hardwareMap.touchSensor.get(limitsUpper[i]);
        }

        // Add a 2 X _ matrix to store the arrays of upper and lower limit switches
        limitSwitchArrays.put(motor, new TouchSensor[][]{sensorsLower, sensorsUpper});
        limitedMotorInfo.put(motor, brake);
    }

    // Move a basic limited motor indefinitely
    public void moveLimitedMotor(String motor, double motorPower){
        int sign = (motorPower < 0 ? -1 : 1);

        // If the motor power is positive, get the upper switch; if it is negative, get the lower switch
        TouchSensor sensor = limitSwitches.get(motor)[sign < 0 ? 0 : 1];
        DcMotor motorToMove = dcMotors.get(motor);
        boolean brake = limitedMotorInfo.get(motor);

        //Wait for sensor to be pressed before stopping
        if(sensor.isPressed()){
            // Stop the motor by braking
            if(!brake) motorToMove.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorToMove.setPower(0);

            // Set the zero power behaviour back to float if the motor is not meant to break
            if(!brake) motorToMove.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }else{
            motorToMove.setPower(motorPower);
        }
    }

    //
    public void moveLimitedMotor(String motor, double distanceCM, double motorPower){
        int sign = (motorPower < 0 ? -1 : 1);

        TouchSensor sensor = limitSwitches.get(motor)[sign < 0 ? 0 : 1];
        DcMotor motorToMove = dcMotors.get(motor);
        boolean brake = limitedMotorInfo.get(motor);

        Double[] info = dcMotorInfo.get(motor);
        double circumference = info[0];
        double encoderTicks = info[1];
        int target = (int) (encoderTicks * distanceCM / circumference);

        double l = 7.62;
        double m = 4/3 * circumference/(l*encoderTicks);
        double dn = 0.0001;
        double slow = l*motorPower*encoderTicks/circumference;

        motorToMove.setTargetPosition(sign * target + dcMotors.get(motor).getCurrentPosition());

        while(Math.abs(motorToMove.getTargetPosition() - motorToMove.getCurrentPosition()) > 10 && !sensor.isPressed()){
            double diff = Math.abs(motorToMove.getTargetPosition() - motorToMove.getCurrentPosition());
            double dp = (diff <= slow && motorPower > 0) ? m*dn : 0;
            motorPower -= dp;

            motorToMove.setPower(motorPower);
        }

        if(sensor.isPressed() && !brake){
            motorToMove.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorToMove.setPower(0);
            motorToMove.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }else{
            motorToMove.setPower(0);
        }
    }

    public void moveLimitedMotorArray(String motor, double motorPower, LimitBehavior behavior){
        int sign = (motorPower <= 0 ? -1 : 1);

        TouchSensor[] sensors = limitSwitchArrays.get(motor)[sign < 0 ? 0 : 1];
        DcMotor motorToMove = dcMotors.get(motor);
        boolean brake = limitedMotorInfo.get(motor);

        if(arePressed(sensors, behavior)){
            if(!brake) motorToMove.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorToMove.setPower(0);

            if(!brake) motorToMove.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }else{
            motorToMove.setPower(motorPower);
        }
    }

    public void moveLimitedMotorArray(String motor, double distanceCM, double motorPower, LimitBehavior behavior) {
        int sign = (motorPower < 0 ? -1 : 1);

        TouchSensor[] sensors = limitSwitchArrays.get(motor)[sign < 0 ? 0 : 1];
        DcMotor motorToMove = dcMotors.get(motor);
        boolean brake = limitedMotorInfo.get(motor);

        Double[] info = dcMotorInfo.get(motor);
        double circumference = info[0];
        double encoderTicks = info[1];
        int target = (int) (encoderTicks * distanceCM / circumference);

        double l = 7.62;
        double m = 4 / 3 * circumference / (l * encoderTicks);
        double dn = 0.0001;
        double slow = l * motorPower * encoderTicks / circumference;

        motorToMove.setTargetPosition(sign * target + dcMotors.get(motor).getCurrentPosition());

        while (Math.abs(motorToMove.getTargetPosition() - motorToMove.getCurrentPosition()) > 10 && arePressed(sensors, behavior)) {
            double diff = Math.abs(motorToMove.getTargetPosition() - motorToMove.getCurrentPosition());
            double dp = (diff <= slow && motorPower > 0) ? m * dn : 0;
            motorPower -= dp;

            motorToMove.setPower(motorPower);
        }

        if(arePressed(sensors, behavior) && !brake){
            motorToMove.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorToMove.setPower(0);

            motorToMove.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }else{
            motorToMove.setPower(0);
        }
    }

    private boolean arePressed(TouchSensor[] sensors, LimitBehavior behavior){
        boolean pressed = behavior == LimitBehavior.AND;

        for(TouchSensor sensor : sensors){
            if(behavior == LimitBehavior.AND) pressed = sensor.isPressed() && pressed;
            if(behavior == LimitBehavior.OR) pressed = sensor.isPressed() || pressed;
        }

        return pressed;
    }

    public void resetLimitedMotor(String motor){
        TouchSensor endSensor = limitSwitches.get(motor)[1];
        DcMotor motorToReset = dcMotors.get(motor);

        while(!endSensor.isPressed()){
            motorToReset.setPower(-0.2);
        }

        motorToReset.setPower(0);
        motorToReset.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetLimitedMotorArray(String motor, LimitBehavior behavior) {
        TouchSensor[] endSensors = limitSwitchArrays.get(motor)[1];
        DcMotor motorToReset = dcMotors.get(motor);

        while(arePressed(endSensors, behavior)){
            motorToReset.setPower(-0.2);
        }

        motorToReset.setPower(0);
        motorToReset.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void addServo(String motor){
        addServo(motor, 180, 180, 0);
    }

    public void addServo(String motor, double startAngle){
        servoPositions.put(motor, startAngle);

        addServo(motor);
    }

    public void addServo(String motor, double rotationAngle, double maxAngle, double minAngle, double startAngle){
        servoPositions.put(motor, startAngle);

        addServo(motor, rotationAngle, maxAngle, minAngle);
    }

    public void addServo(String motor, double rotationAngle, double maxAngle, double minAngle){
        servos.put(motor, hardwareMap.servo.get(motor));
        servoLimits.put(motor, new Double[]{rotationAngle, minAngle, maxAngle});
    }

    public void resetServo(String motor, int waitTimeMillis){
        Double startAngle = servoPositions.get(motor);

        servos.get(motor).setPosition((startAngle == null) ? servoLimits.get(motor)[1] : startAngle);
        waitMillis(waitTimeMillis);
    }

    public void incrementServo(String motor, double increment, int waitTimeMillis){
        Double startAngle = servoPositions.get(motor);
        double target = startAngle == null ? 0 : startAngle + increment;

        Double[] limits = servoLimits.get(motor);

        if(target > limits[2]) target = limits[2];
        if(target < limits[1]) target = limits[1];

        if(startAngle != null){
            rotateServo(motor, target, waitTimeMillis);
        }
    }

    public double getServoPos(String motor){
        Double startAngle = servoPositions.get(motor);

        if(startAngle != null){
            return startAngle;
        }else{
            return 0;
        }
    }

    public void rotateServo(String motor, double angle, int waitTimeMillis){
        Double[] angleLimits = servoLimits.get(motor);

        double rotationAngle = angleLimits[0];
        double minAngle = angleLimits[1];
        double maxAngle = angleLimits[2];

        if(angle >= minAngle && angle <= maxAngle){
            servos.get(motor).setPosition(angle/rotationAngle);

            Double currentAngle = servoPositions.get(motor);
            if(currentAngle != null) servoPositions.put(motor, angle);

            waitMillis(waitTimeMillis);
        }
    }

    public void holdServo(String motor, double angle, int waitTimeMillis){
        long start = System.currentTimeMillis();

        while(System.currentTimeMillis() - start < waitTimeMillis){
            rotateServo(motor, angle, 0);
        }
    }

    public void waitMillis(int millis) {
        try {
            Thread.sleep(millis);
        } catch(InterruptedException e) { }
    }

    public void addColorSensor(String sensor, int scaleFactor){
        colorSensors.put(sensor, hardwareMap.get(ColorSensor.class, sensor));
        colorSensorInfo.put(sensor, scaleFactor);
    }

    public int[] getColorRGBA(String sensor){
        ColorSensor sensorToUse = colorSensors.get(sensor);
        int scaleFactor = colorSensorInfo.get(sensor);

        return new int[]{
                scaleFactor*sensorToUse.red(),
                scaleFactor*sensorToUse.green(),
                scaleFactor*sensorToUse.blue(),
                scaleFactor*sensorToUse.alpha()
        };
    }

    public void addDistanceSensor(String sensor, boolean optical){
        if(optical){
            distanceSensors.put(sensor, (DistanceSensor) hardwareMap.opticalDistanceSensor.get(sensor));
        }else{
            distanceSensors.put(sensor, hardwareMap.get(DistanceSensor.class, sensor));
        }
    }

    public double getDistanceCM(String sensor){
        DistanceSensor sensorToUse = distanceSensors.get(sensor);

        return sensorToUse.getDistance(DistanceUnit.CM);
    }

    private void initVuforia(String vuforiaKey, VuforiaLocalizer.CameraDirection camera){
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();

        params.vuforiaLicenseKey = vuforiaKey;
        params.cameraDirection = camera;

        vuforia = ClassFactory.getInstance().createVuforia(params);
    }

    private void initTfod(String tfodModelAsset, String[] labels){
        int tfodMonitorViewID = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
        );

        TFObjectDetector.Parameters params = new TFObjectDetector.Parameters(tfodMonitorViewID);
        params.minResultConfidence = 0.8f;

        tfod = ClassFactory.getInstance().createTFObjectDetector(params, vuforia);

        tfod.loadModelFromAsset(tfodModelAsset, labels);
    }

    // Initialize TensorFlow and Vuforia
    public void initCV(String vuforiaKey, VuforiaLocalizer.CameraDirection camera, String tfodModelAsset, String[] labels){
        initVuforia(vuforiaKey, camera);
        initTfod(tfodModelAsset, labels);

        if(tfod != null){
            isCVReady = true;
            tfod.activate();
        }else{
            telemetry.addData("Error:", "This device is not compatible with TFOD");
        }
    }

    public void shutDownCV(){
        if(isCVReady){
            tfod.shutdown();
        }
    }

    public Recognition recognize(String label) throws CVInitializationException {
        if(isCVReady){
            Recognition matched = null;
            List<Recognition> recognitions = tfod.getUpdatedRecognitions();

            if(recognitions != null){
                for(Recognition recognition : recognitions){
                    if(recognition.getLabel().equals(label)){
                        matched = recognition;
                        break;
                    }
                }
            }else{
                matched = null;
            }

            return matched;
        }else{
            throw new CVInitializationException();
        }
    }

    public List<Recognition> recognizeAll(String label) throws CVInitializationException {
        if(isCVReady){
            List<Recognition> matched = new ArrayList<>();
            List<Recognition> recognitions = tfod.getUpdatedRecognitions();

            for(Recognition recognition : recognitions){
                if(recognition.getLabel().equals(label)){
                    matched.add(recognition);
                }
            }

            return matched;
        }else{
            throw new CVInitializationException();
        }
    }

    public void autoAlign(String targetLabel, double centerPoint) throws CVInitializationException {
        // Coefficient that multiplies error for rotation speed (maybe implement PID later)
        double p = 0.5;
        // Threshold for the target to be "close enough" to centered, measured as proportion of total image width
        double threshold = 1e-3;
        // Initialize the error variable
        double error = getError(targetLabel, centerPoint);

        while(Math.abs(error) > threshold){
            telemetry.addData("error:", error);
            telemetry.update();

            drive(1, p*error, 0, 0);
            stop();
            error = getError(targetLabel, centerPoint);
        }

        stop();
    }

    // Output target distance from center point as a proportion to overall image size
    private double getError(String targetLabel, double centerPoint) throws CVInitializationException {
        Recognition target = recognize(targetLabel);

        if(target != null){
            double x1 = target.getLeft();
            double x2 = target.getRight();

            double unscaled = (x1 + x2)/2 - centerPoint;

            return unscaled/target.getImageWidth();
        }else{
            return 1;
        }
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public static enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT;
    }

    public static enum LimitBehavior {
        AND, OR
    }

    public static interface Action {
        public void run(Robot bot);
    }

    // All public CV methods should throw this exception if !isCVReady
    public static class CVInitializationException extends Exception {
        // Exception to explain CV is not initialized
        public CVInitializationException(){
            super("Please use the initCV method before invoking TensorFlow or Vuforia functions");
        }
    }

    public static class ButtonState {
        public boolean a = false;
        public boolean b = false;
        public boolean x = false;
        public boolean y = false;

        public void update(boolean aNew, boolean bNew, boolean xNew, boolean yNew){
            a = aNew;
            b = bNew;
            x = xNew;
            y = yNew;
        }
    }

    public static class DpadState {
        public boolean dpad_up = false;
        public boolean dpad_down = false;
        public boolean dpad_right = false;
        public boolean dpad_left = false;

        public void update(boolean dpad_upNew, boolean dpad_downNew, boolean dpad_rightNew, boolean dpad_leftNew){
            dpad_up = dpad_upNew;
            dpad_down = dpad_downNew;
            dpad_right = dpad_rightNew;
            dpad_left = dpad_leftNew;
        }
    }

    public static class BumperState {
        public boolean right_bumper = false;
        public boolean left_bumper = false;

        public void update(boolean bumper_rightNew, boolean bumper_leftNew) {
            right_bumper = bumper_rightNew;
            left_bumper = bumper_leftNew;
        }
    }
}
