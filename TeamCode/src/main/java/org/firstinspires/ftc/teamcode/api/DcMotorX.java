package org.firstinspires.ftc.teamcode.api;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DcMotorX {

    public DcMotor core;
    public int ticksPerRev;
    public double distancePerRev;

    private double lastPosition = 0;
    private double currentPosition = 0;

    public DcMotorX(DcMotor motor){
        this(motor, 0, 0);
    }

    public DcMotorX(DcMotor motor, int ticksPerRev, double distancePerRev){
        this.core = motor;
        this.ticksPerRev = ticksPerRev;
        this.distancePerRev = distancePerRev;
    }

    private int getEncoderPosition(double distance){
        return (int) Math.round(ticksPerRev * distance / distancePerRev);
    }

    private double getDistanceFrom(int encoderPosition){
        return distancePerRev * encoderPosition / ticksPerRev;
    }

    public void controlVelocity(){
        core.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void controlPosition(){
        core.setTargetPosition(0);
        core.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoder(){
        core.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runWithoutEncoder(){
        core.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void reverse(){
        DcMotorSimple.Direction direction = core.getDirection();

        if(direction.equals(DcMotorSimple.Direction.REVERSE)){
            core.setDirection(DcMotorSimple.Direction.FORWARD);
        }else{
            core.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public double getPower(){
        return core.getPower();
    }
    
    public double getVelocity() { return getPower(); }

    public void setPower(double power){
        core.setPower(power);
    }

    public double getPosition(){
        currentPosition = getDistanceFrom(core.getCurrentPosition());
        return currentPosition;
    }

    public double getPositionForward(){
        if(core.getDirection() == DcMotorSimple.Direction.REVERSE) return -getPosition();
        else return getPosition();
    }

    public void savePosition(boolean useCurrent){
        if(useCurrent) lastPosition = currentPosition;
        else lastPosition = getPosition();
    }

    public double getDistance(boolean useCurrent){
        if(useCurrent) return currentPosition - lastPosition;
        else return getPosition() - lastPosition;
    }

    public void setVelocity(double velocity){
        core.setPower(velocity);
    }

    public void setPosition(double position, double speed){
        core.setTargetPosition(getEncoderPosition(position));
        core.setPower(speed);
    }

    public void setDistance(double distance, double speed){
        setPosition(getPosition() + distance, speed);
    }

    public void goToPosition(double position, double speed){
        if(!core.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) controlPosition();
        setPosition(position, speed);

        while(core.isBusy());
    }

    public void goToDistance(double distance, double speed){
        goToPosition(distance + getPosition(), speed);
    }

    public static double getControlledSpeed(double maxSpeed, double threshold, double error, boolean useThreshold){
        double speed = Math.signum(error)*maxSpeed;

        if(Math.abs(error) < 2*threshold){
            speed *= error / (2*threshold);
        }

        if(Math.abs(error) < threshold && useThreshold){
            speed = 0;
        }

        return speed;
    }

    public double getTargetPosition(){
        return getDistanceFrom(core.getTargetPosition());
    }

    public void setBrake(boolean brake){
        if(brake){
            core.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }else{
            core.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    protected void setTargetPosition(double targetPosition){ core.setTargetPosition(getEncoderPosition(targetPosition)); }

    protected void setTargetDistance(double targetDistance){ setTargetPosition(getPosition() + targetDistance); }
}
