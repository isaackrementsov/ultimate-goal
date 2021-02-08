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
    private double backDistancePerDegree;
    // Width between the left and right encoders
    private double width;

    // Position coordinates (x, y, heading [phi])
    public double x;
    public double y;
    public double phi;

    public Odometry(DcMotorX wheelR, DcMotorX wheelL, DcMotorX wheelB, int cycleTime, double backDistancePerDegree, double width, double x0, double y0, double phi0){
        this.cycleTime = cycleTime;

        this.wheelR = wheelR;
        this.wheelL = wheelL;
        this.wheelB = wheelB;

        this.backDistancePerDegree = backDistancePerDegree;
        this.width = width;

        this.x = x0;
        this.y = y0;
        this.phi = phi0;
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

    public void update(){
        // Changes in position measured by the encoders
        double dR = wheelR.getDisplacement(false);
        double dL = wheelL.getDisplacement(false);
        double dB = wheelB.getDisplacement(false);

        // Save positions to get position changes later
        wheelR.savePosition(true);
        wheelL.savePosition(true);
        wheelB.savePosition(true);

        // Calculate change in heading
        double dphi = (dR - dL)/width;

        // Use this to find linear and perpendicular motion
        double dS = arcdS(dR, dL, dphi);
        double dP = dB - backDistancePerDegree*dphi;

        // Add components of the linear and perpendicular motion to update position
        x += dS*Math.cos(phi + dphi/2) + dP*Math.sin(phi + dphi/2);
        y += dS*Math.sin(phi + dphi/2) - dP*Math.cos(phi + dphi/2);
        phi += dphi;
    }

    public void run(){
        // Run the thread indefinitely
        while(isRunning){
            try {
                // Wait for about a hardware cycle
                Thread.sleep(cycleTime);
            }catch(Exception e){
                e.printStackTrace();
            }

            // Update position coordinate
            update();
        }

    }

    // Stop the thread
    public void stop(){ isRunning = false; }
}
