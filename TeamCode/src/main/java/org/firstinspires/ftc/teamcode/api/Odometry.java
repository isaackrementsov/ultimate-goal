/*
FTC Odometry Code
Works with a three-point dead wheel system and uses circular arc approximation

Written by Isaac Krementsov, 12/1/2020
*/

package org.firstinspires.ftc.teamcode.api;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Odometry implements Runnable {
    // Run/Stop Odometry thread
    private boolean isRunning = true;
    // Amount of time to wait for hardware cycles
    private int cycleTime;

    // Encoder "dead" wheels (Right, Left, and Back)
    public DcMotorX wheelR, wheelL, wheelB;
    // Ticks read by the back encoder per degree of rotation (measure experimentally)
    private double backDistancePerRadian;
    // Width between the left and right encoders
    private double width;

    private double lastR;
    private double lastL;
    private double lastB;

    // Position coordinates (x, y, heading [phi])
    public double x;
    public double y;
    public double phi;

    public long actualTime;

    public Odometry(DcMotorX wheelR, DcMotorX wheelL, DcMotorX wheelB, int cycleTime, double backDistancePerRadian, double width, double x0, double y0, double phi0){
        this.cycleTime = cycleTime;

        this.wheelR = wheelR;
        this.wheelL = wheelL;
        this.wheelB = wheelB;

        this.backDistancePerRadian = backDistancePerRadian;
        this.width = width;

        this.x = x0;
        this.y = y0;
        this.phi = phi0;

        this.actualTime = cycleTime;
    }

    public double arcdS(double dR, double dL, double dphi){
        // Calculate the robot's total linear displacement, assuming it moves in an arc
        if(dphi == 0){
            // The limit of the full formula as dphi approaches 0
            return (dR + dL)/2;
        }else{
            return (dR + dL)*Math.sin(dphi/2)/dphi;
        }
    }

    public void update() {
        long start = System.currentTimeMillis();

        double R = wheelR.getPosition();
        double L = wheelL.getPosition();
        double B = wheelB.getPosition();

        double dR = R - lastR;
        double dL = L - lastL;
        double dB = B - lastB;

        lastR = R;
        lastL = L;
        lastB = B;

        // Calculate change in heading
        double dphi = (dL - dR) / width;

        // Use this to find linear and perpendicular motion
        double dS = arcdS(dR, dL, dphi);
        double dP = dB - backDistancePerRadian * dphi;

        // Add components of the linear and perpendicular motion to update position
        x += dS * Math.sin(phi + dphi/2) - dP * Math.cos(phi + dphi/2);
        y += -dS * Math.cos(phi + dphi/2) - dP * Math.sin(phi + dphi/2);
        phi += dphi;

        actualTime = System.currentTimeMillis() - start;
    }

    public void run(){
        lastR = wheelR.getPosition();
        lastL = wheelL.getPosition();
        lastB = wheelB.getPosition();

        // Run the thread indefinitely
        while(isRunning){
            // Update position coordinate
            update();

            if(actualTime < cycleTime){
                try {
                    Thread.sleep(cycleTime - actualTime);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

        }

    }

    public void reset(){
        x = 0;
        y = 0;
        phi = 0;
    }

    public void reset(double... coords){
        x = coords[0];
        y = coords[1];
        phi = coords[2];
    }

    // Stop the thread
    public void stop(){ isRunning = false; }
}
