package org.firstinspires.ftc.teamcode.api;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.List;

public class LimitedMotorX extends DcMotorX {

    private TouchSensor limitLower = null;
    private TouchSensor limitUpper = null;

    private Double positionLimitLower = null;
    private Double positionLimitUpper = null;

    public LimitedMotorX(DcMotor motor){
        super(motor, 0, 0);
    }

    public LimitedMotorX(DcMotor motor, int ticksPerRev, double distancePerRev){
        super(motor, ticksPerRev, distancePerRev);
    }

    public void setLowerLimit(TouchSensor limitLower){
        this.limitLower = limitLower;
    }

    public void setLowerLimit(double positionLimitLower) {
        this.positionLimitLower = positionLimitLower;
    }

    public void setUpperLimit(TouchSensor limitUpper){
        this.limitUpper = limitUpper;
    }

    public void setUpperLimit(double positionLimitUpper){
        this.positionLimitUpper = positionLimitUpper;
    }

    public void setLimits(TouchSensor limitLower, TouchSensor limitUpper){
        setLowerLimit(limitLower);
        setUpperLimit(limitUpper);
    }

    public void setLimits(double positionLimitLower, double positionLimitUpper){
        setLowerLimit(positionLimitLower);
        setUpperLimit(positionLimitUpper);
    }

    public void setLimits(TouchSensor limitLower, double positionLimitUpper){
        setLowerLimit(limitLower);
        setUpperLimit(positionLimitUpper);
    }

    public void setLimits(double positionLimitLower, TouchSensor limitUpper){
        setLowerLimit(positionLimitLower);
        setUpperLimit(limitUpper);
    }

    public boolean limitPressed(){
        return limitLowerPressed() || limitUpperPressed();
    }

    public boolean limitPressed(double targetPosition){
        return targetPosition > getPosition() ? limitUpperPressed() : limitLowerPressed();
    }

    public boolean limitLowerPressed(){
        if(limitLower == null){
            if(positionLimitLower == null){
                return false;
            }else{
                return getPosition() >= positionLimitLower;
            }
        }else{
            return limitLower.isPressed();
        }
    }

    public boolean limitUpperPressed(){
        if(limitUpper == null){
            if(positionLimitUpper == null){
                return false;
            }else{
                return getPosition() <= positionLimitUpper;
            }
        }else{
            return limitLower.isPressed();
        }
    }

    public void goToDistance(double position, double speed){
        if(!core.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) controlPosition();
        setDistance(position, speed);

        while(core.isBusy() && !limitPressed(position));
    }

    public void goToPosition(double position, double speed){
        if(!core.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) controlPosition();
        setPosition(position, speed);

        while(core.isBusy() && !limitPressed(position));
    }

    public void goToLowerSwitch(double speed){
        setPower(speed);
        while(!limitLower.isPressed());
        setPower(0);
    }

    public void reset(){ reset(0.2); }

    public void reset(double speed){
        goToLowerSwitch(speed);
    }

    public void goToUpperSwitch(double speed){
        setPower(speed);
        while(!limitUpper.isPressed());
        setPower(0);
    }

}
