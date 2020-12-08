/*
Drivetrain Class
Provides easy control for a Mecanum drivetrain (feel free to modify this if using another system)

Written by Isaac Krementsov, 12/7/2020
*/

package org.firstinspires.ftc.teamcode.api;

public class Drivetrain {

    DcMotorX mRF, mLF, mRB, mLB;

    public Drivetrain(DcMotorX mRF, DcMotorX mLF, DcMotorX mRB, DcMotorX mLB, boolean reverseLeft){
        this.mRF = mRF;
        this.mLF = mLF;
        this.mRB = mRB;
        this.mLB = mLB;

        if(reverseLeft){
            this.mLF.reverse();
            this.mLB.reverse();
        }
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

    public void rotate(double power){
        setPowerRight(power);
        setPowerLeft(-power);
    }

    public void strafe(double power){
        mRF.setPower(-power);
        mLF.setPower(power);
        mRB.setPower(power);
        mLB.setPower(-power);
    }

    public void driveWithGamepad(double speed, double forward, double yaw, double strafe){
        drive(speed*forward, -speed*yaw, -speed*strafe);
    }

    public void drive(double power){
        setPowerAll(power);
    }

    public void drive(double power, double yaw, double strafe){
        mRF.setPower(power + yaw + strafe);
        mLF.setPower(power - yaw - strafe);
        mRB.setPower(power + yaw - strafe);
        mLB.setPower(power - yaw + strafe);
    }

    public void stop(){
        setPowerRight(0);
        setPowerLeft(0);
    }

}
