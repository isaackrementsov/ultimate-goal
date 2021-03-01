/*
Drivetrain Class
Provides easy control for a Mecanum drivetrain (feel free to modify this if using another system)

Written by Isaac Krementsov, 12/7/2020
*/

package org.firstinspires.ftc.teamcode.api;

public class Drivetrain {

    public DcMotorX mRF, mLF, mRB, mLB;

    public Drivetrain(DcMotorX mRF, DcMotorX mLF, DcMotorX mRB, DcMotorX mLB){
        this.mRF = mRF;
        this.mLF = mLF;
        this.mRB = mRB;
        this.mLB = mLB;

        reverseLeft();
        setBrake(true);
    }

    private void setPowerRight(double power){
        mRF.setPower(power);
        mRB.setPower(power);
    }

    private void setPowerLeft(double power){
        mLF.setPower(power);
        mLB.setPower(power);
    }

    private void setPowerAll(double power){
        setPowerRight(power);
        setPowerLeft(power);
    }

    public void reverse(){
        mRF.reverse();
        mRB.reverse();
        mLF.reverse();
        mLB.reverse();
    }

    public void reverseLeft(){
        mLF.reverse();
        mLB.reverse();
    }

    public void driveWithGamepad(double speed, double forward, double yaw, double strafe){
        drive(speed*forward, -speed*yaw, speed*strafe);
    }

    public void drive(double power, double yaw, double strafe){
        mRF.setPower(power + yaw - strafe);
        mLF.setPower(power - yaw + strafe);
        mRB.setPower(power + yaw + strafe);
        mLB.setPower(power - yaw - strafe);
    }

    public void drive(double power, Direction direction){
        switch(direction){
            case FORWARD:
                drive(power, 0, 0);
                break;
            case BACKWARD:
                drive(-power, 0, 0);
                break;
            case RIGHT:
                drive(0, 0, -power);
                break;
            case LEFT:
                drive(0, 0, power);
                break;
        }
    }

    // Set all motors to position control mode
    public void controlPosition(){
        mRF.controlPosition();
        mLF.controlPosition();
        mRB.controlPosition();
        mLB.controlPosition();
    }

    public void drive(double power, double distance, Direction direction, boolean blocking) {
        // Set the correct target distances based on drive direction
        switch(direction){
            case FORWARD:
                mRF.setTargetDistance(distance);
                mLF.setTargetDistance(distance);
                mRB.setTargetDistance(distance);
                mLB.setTargetDistance(distance);
            case BACKWARD:
                mRF.setTargetDistance(-distance);
                mLF.setTargetDistance(-distance);
                mRB.setTargetDistance(-distance);
                mLB.setTargetDistance(-distance);
            case RIGHT:
                mRF.setTargetDistance(-distance);
                mLF.setTargetDistance(distance);
                mRB.setTargetDistance(distance);
                mLB.setTargetDistance(-distance);
            case LEFT:
                mRF.setTargetDistance(distance);
                mLF.setTargetDistance(-distance);
                mRB.setTargetDistance(-distance);
                mLB.setTargetDistance(distance);
        }

        // Set the right motor powers
        drive(power, direction);

        // Wait for the motors to finish if blocking
        if(blocking){
            // Any of the motors can finish for the loop to stop
            while(mRF.core.isBusy() && mLF.core.isBusy() && mRB.core.isBusy() && mLB.core.isBusy());
        }
    }

    // Stop all motors
    public void stop(){
        setPowerAll(0);
    }

    public void setBrake(boolean brake){
        if(brake){
            mRF.setBrake(true);
            mLF.setBrake(true);
            mRB.setBrake(true);
            mLB.setBrake(true);
        }else{
            mRF.setBrake(false);
            mLF.setBrake(false);
            mRB.setBrake(false);
            mLB.setBrake(false);
        }
    }

    public static enum Direction {
        FORWARD, BACKWARD, RIGHT, LEFT
    }

}
