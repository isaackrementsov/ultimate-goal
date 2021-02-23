/*
PID Drivetrain Controller (uses Odometry for position tracking)
Written by Isaac Krementsov, 12/7/2020
*/

package org.firstinspires.ftc.teamcode.api;

public class ControlledDrivetrain extends Drivetrain implements Runnable {

    // PID tuning parameters (9 total, 3 for each coordinate)
    private double[] Kp;
    private double[] Ki;
    private double[] Kd;
    // Time to wait between updates/cycles (in milliseconds)
    private int cycleTime;
    // Time that each update actually takes (in seconds), used to compute integrals/derivatives wrt time
    private double dt;

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
                0.01, 0.01, 0.01
        );
    }

    // Constructor that allows thresholds to be changed
    public ControlledDrivetrain(DcMotorX mRF, DcMotorX mLF, DcMotorX mRB, DcMotorX mLB, Odometry positionTracker, double xThreshold, double yThreshold, double phiThreshold){
        this(
                mRF, mLF, mRB, mLB,
                positionTracker,
                xThreshold, yThreshold, phiThreshold,
                new double[]{1,1,1}, new double[]{0,0,0}, new double[]{0,0,0}, 50
        );
    }

    // Full constructor with tuning options
    public ControlledDrivetrain(DcMotorX mRF, DcMotorX mLF, DcMotorX mRB, DcMotorX mLB, Odometry positionTracker, double xThreshold, double yThreshold, double phiThreshold, double[] Kp, double[] Ki, double[] Kd, int cycleTime){
        super(mRF, mLF, mRB, mLB);

        mRF.runWithoutEncoder();
        mLF.runWithoutEncoder();
        mRB.runWithoutEncoder();
        mLB.runWithoutEncoder();

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
        this.dt = cycleTime * 1.0 / 1000.0;
    }

    // Main PID Control Loop
    public void update(){
        // Only correct the robot's position when active
        if(active){
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
            double dsdt = Cx*Math.sin(positionTracker.phi) - Cy*Math.cos(positionTracker.phi);
            // Speed at which the robot should move sideways (strafe) (calculated from x and y corrections using a rotation matrix)
            double dpdt = Cy*Math.sin(positionTracker.phi) - Cx*Math.cos(positionTracker.phi);
            // Speed at which the robot should rotate (change its heading)
            double dphidt = Cphi;

            // Drive the robot in the correct direction and at the correct speed
            drive(dsdt, dphidt, dpdt);

            // Add to the error integrals
            IEx += Ex*dt;
            IEy += Ey*dt;
            IEphi += IEphi*dt;

            // Save the current error values for the next cycle's derivative calculation
            ExL = Ex;
            EyL = Ey;
            EphiL = Ephi;
        }
    }

    // Set a target position
    public void setPosition(double x, double y, double phi){
        xT = x;
        yT = y;
        phiT = phi;

        // Get initial error measurements
        ExL = xT - positionTracker.x;
        EyL = yT - positionTracker.y;
        EphiL = phiT - positionTracker.phi;
    }

    public boolean isBusy(){
        double Ex = xT - positionTracker.x;
        double Ey = yT - positionTracker.y;
        double Ephi = phiT - positionTracker.phi;

        return Math.abs(Ex) <= xThreshold && Math.abs(Ey) <= yThreshold && Math.abs(Ephi) < phiThreshold;
    }

    // Run the update() loop continuously
    public void run(){
        // Track position on a separate thread
        Thread odometryThread = new Thread(positionTracker);
        odometryThread.start();

        // Run the thread indefinitely
        while(isRunning){
            double start = System.nanoTime();

            try {
                // Wait for about a hardware cycle
                Thread.sleep(cycleTime);
            }catch(Exception e){
                e.printStackTrace();
            }

            // This is normally around cycleTime, but it helps to be exact
            dt = (System.nanoTime() - start)/1e9;
            // Update position coordinate
            update();
        }
    }

    // Stop the controller & odometry code from running
    public void stop(){
        positionTracker.stop();
        isRunning = false;
    }

    // Activate/deactive position correction
    public void setActive(boolean active){ this.active = active; }
}
