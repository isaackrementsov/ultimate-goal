/*
PID Drivetrain Controller (uses Odometry for position tracking)
Written by Isaac Krementsov, 12/7/2020
*/

package org.firstinspires.ftc.teamcode.api;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ControlledDrivetrain extends Drivetrain implements Runnable {

    // PID tuning parameters (9 total, 3 for each coordinate)
    public double[] Kp;
    public double[] Ki;
    public double[] Kd;
    // Time to wait between updates/cycles (in milliseconds)
    private int cycleTime;
    private int actualTime;
    // Time that each update actually takes (in seconds), used to compute integrals/derivatives wrt time
    private double dt;

    // For development purposes only
    public Telemetry telemetry;

    // Threshold to determine whether the robot is close enough to the target position
    private double xThreshold;
    private double yThreshold;
    private double phiThreshold;

    // Setpoint
    public double xT;
    public double yT;
    public double phiT;

    // Error values from the last cycle (used for computing error derivatives)
    public double ExL = 0;
    public double EyL = 0;
    public double EphiL = 0;

    // Integrals of error wrt time
    private double IEx = 0;
    private double IEy = 0;
    private double IEphi = 0;

    // Whether the drivetrain should actively correct the robot's position
    private boolean active = false;
    // Whether the entire thread should continue running
    private boolean isRunning = true;

    // Odometry thread for tracking position
    public Odometry positionTracker;

    // Basic constructor (no tuning options or custom thresholds)
    public ControlledDrivetrain(DcMotorX mRF, DcMotorX mLF, DcMotorX mRB, DcMotorX mLB, Odometry positionTracker){
        this(
                mRF, mLF, mRB, mLB,
                positionTracker,
                2, 2, 2*Math.PI/180
        );
    }

    // TODO: Make isBusy dependent on error derivative

    // Constructor that allows thresholds to be changed
    public ControlledDrivetrain(DcMotorX mRF, DcMotorX mLF, DcMotorX mRB, DcMotorX mLB, Odometry positionTracker, double xThreshold, double yThreshold, double phiThreshold){
        this(
                mRF, mLF, mRB, mLB,
                positionTracker,
                xThreshold, yThreshold, phiThreshold,
                new double[]{0.12,0.05,2.48}, new double[]{0.001,0,0.01}, new double[]{0.01209,0.012,0.14}, 50
        );
    }

    // Full constructor with tuning options
    public ControlledDrivetrain(DcMotorX mRF, DcMotorX mLF, DcMotorX mRB, DcMotorX mLB, Odometry positionTracker, double xThreshold, double yThreshold, double phiThreshold, double[] Kp, double[] Ki, double[] Kd, int cycleTime){
        super(mRF, mLF, mRB, mLB);

        // Add the position tracker and start the target coordinates at the initial reading
        this.positionTracker = positionTracker;

        // Set thresholds
        this.xThreshold = xThreshold;
        this.yThreshold = yThreshold;
        this.phiThreshold = phiThreshold;

        // PID coefficients
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        // Cycle time in milliseconds
        this.cycleTime = cycleTime;
        // Initially approximate dt as cycle time in seconds (it will be measured later)
        this.dt = toSec(cycleTime);
    }

    // Main PID Control Loop
    public void update(){
        long start = System.currentTimeMillis();

        // Error from target for each coordinate
        double Ex =  xT - positionTracker.x;
        double Ey = yT - positionTracker.y;
        double Ephi = phiT - positionTracker.phi;

        // Time derivative of each coordinate's error
        double dExdt = (Ex - ExL)/dt;
        double dEydt = (Ey - EyL)/dt;
        double dEphidt = (Ephi - EphiL)/dt;

        // PID Correction that needs to be made to each coordinate
        double Cx = Kp[0]*Ex + Ki[0]*IEx + Kd[0]*dExdt;
        double Cy = Kp[1]*Ey + Ki[1]*IEy + Kd[1]*dEydt;
        double Cphi = Kp[2]*Ephi + Ki[2]*IEphi + Kd[2]*dEphidt;

        // Speed at which the robot should move forward/in reverse (calculated from x and y corrections using a rotation matrix)
        double dsdt = -Cx*Math.sin(positionTracker.phi) + Cy*Math.cos(positionTracker.phi);
        // Speed at which the robot should move sideways (strafe) (calculated from x and y corrections using a rotation matrix)
        double dpdt = Cy*Math.sin(positionTracker.phi) + Cx*Math.cos(positionTracker.phi);
        // Speed at which the robot should rotate (change its heading)
        double dphidt = Cphi;

        // Drive the robot in the correct direction and at the correct speed
        // Only correct the robot's position when active
        if(active) drive(dsdt, dphidt, dpdt);

        // Add to the error integrals
        IEx += Ex*dt;
        IEy += Ey*dt;
        IEphi += Ephi*dt;

        // Save the current error values for the next cycle's derivative calculation
        ExL = Ex;
        EyL = Ey;
        EphiL = Ephi;

        actualTime = (int) (System.currentTimeMillis() - start);
    }

    // Set a target position
    public void setPosition(double... coords){
        xT = coords[0];
        yT = coords[1];
        phiT = coords[2];

        // Get initial error measurements
        ExL = xT - positionTracker.x;
        EyL = yT - positionTracker.y;
        EphiL = phiT - positionTracker.phi;

        IEx = 0;
        IEy = 0;
        IEphi = 0;
    }

    public boolean isBusy(double xThresh, double yThresh, double phiThres){
        double Ex = xT - positionTracker.x;
        double Ey = yT - positionTracker.y;
        double Ephi = phiT - positionTracker.phi;

        return Math.abs(Ex) > xThresh || Math.abs(Ey) > yThresh || Math.abs(Ephi) > phiThres;
    }

    public boolean isBusy(){
        return isBusy(xThreshold, yThreshold, phiThreshold);
    }

    private double toSec(int millis){
        return 1.0*millis/(1000.0);
    }

    // Run the update() loop continuously
    public void run(){
        // Track position on a separate thread
        Thread odometryThread = new Thread(positionTracker);
        odometryThread.start();

        // Run the thread indefinitely
        while(isRunning){
            update();

            if(actualTime < cycleTime){
                try {
                    // Wait for about a hardware cycle
                    Thread.sleep(cycleTime - actualTime);
                }catch(Exception e){
                    e.printStackTrace();
                }

                dt = toSec(cycleTime);
            }else{
                dt = toSec(actualTime);
            }
        }
    }

    // Stop the controller & odometry code from running
    public void stopController(){
        positionTracker.stop();
        isRunning = false;
    }

    // Activate/deactive position correction
    public void setActive(boolean active){ this.active = active; }

    // Show whether positon correction is active
    public boolean getActive(){ return active; }
}
