package org.firstinspires.ftc.teamcode.api;


import com.qualcomm.robotcore.hardware.Servo;

public class ServoX {

    public Servo core;

    public double rotationAngle;
    public double maxAngle = Double.MAX_VALUE;
    public double minAngle = Double.MIN_VALUE;

    public ServoX(Servo core){ this(core, 180, 180); }
    public ServoX(Servo core, double rotationAngle, double maxAngle){ this(core, rotationAngle, maxAngle, 0); }
    public ServoX(Servo core, double rotationAngle, double maxAngle, double minAngle){ this(core, rotationAngle, maxAngle, minAngle, 0); }

    public ServoX(Servo core, double rotationAngle, double maxAngle, double minAngle, double startAngle){
        this.rotationAngle = rotationAngle;
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        this.core = core;
    }

    public void setDistance(double angle){
        setAngle(angle + getAngle());
    }

    public void goToDistance(double angle, int wait) throws InterruptedException {
        goToAngle(angle, wait);
    }

    public void setAngle(double angle) {
        try {
            goToAngle(angle, 0);
        }catch(InterruptedException _ie){  }
    }

    public void goToAngle(double angle, int wait) throws InterruptedException {
        if(angle <= maxAngle && angle >= minAngle) {
            core.setPosition(angle / rotationAngle);
        }

        if(wait > 0){
            Thread.sleep(wait);
        }
    }

    public double getAngle(){
        return core.getPosition() * rotationAngle;
    }

}
