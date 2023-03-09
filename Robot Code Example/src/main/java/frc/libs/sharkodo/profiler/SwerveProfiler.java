package frc.libs.sharkodo.profiler;

import frc.libs.sharkodo.pose.Twist2D;
import frc.libs.sharkodo.sharkodo.Exceptions;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveProfiler implements Profiler {

    private final Twist2D chassisPose;
    private final double[] previousModuleDrivePositions;
    private final Supplier<double[][]> swerveGetPose;
    private final DoubleSupplier getHeading;

    private final double ticksPerInch;
    private final double delay;

    public SwerveProfiler(Supplier<double[][]> swerveGetPose, DoubleSupplier getHeading, double[] pose, double ticksPerInch, double delay) {
        this(swerveGetPose, getHeading, ticksPerInch, delay);
        chassisPose.setTwist(pose);
    }

    public SwerveProfiler(Supplier<double[][]> swerveGetPose, DoubleSupplier getHeading, double ticksPerInch, double delay) {
        this.swerveGetPose = swerveGetPose;
        this.getHeading = getHeading;
        this.ticksPerInch = ticksPerInch;
        this.delay = delay/1000.0;

        //get pose and determine length here
//        this.previousModuleDrivePositions = new double[4];
        double[][] preliminaryPose = this.swerveGetPose.get();
        previousModuleDrivePositions = new double[preliminaryPose.length];
        for(int i = 0; i < preliminaryPose.length; i++) {
            previousModuleDrivePositions[i] = preliminaryPose[i][0];
        }
        chassisPose = new Twist2D(0, 0, 0, 0);
    }

    @Override
    public Twist2D poll() throws Exceptions.PoseTypeMismatch {
        double[][] pose = swerveGetPose.get();
        double heading = getHeading.getAsDouble();
        double[][] modPose = new double[pose.length][2];

        for(int i = 0; i < pose.length; i++) {
            double distance = pose[i][0] - previousModuleDrivePositions[i];
            double angle = pose[i][1] + heading;
            modPose[i][0] = distance * Math.cos(angle);
            modPose[i][1] = distance * Math.sin(angle);

            SmartDashboard.putNumber("module " + i + " x is ", modPose[i][0]);
            SmartDashboard.putNumber("module " + i + " y is ", modPose[i][1]);
            previousModuleDrivePositions[i] = pose[i][0];
        }

        double x = chassisPose.getX(), y = chassisPose.getY();

        for(double[] p : modPose) {
            x += (1.0/modPose.length) * p[0];
            y += (1.0/modPose.length) * p[1];
        }

        double deltaX = x - chassisPose.getX();
        double deltaY = y - chassisPose.getY();
        double velocity = (Math.hypot(deltaX, deltaY))/delay;

        chassisPose.setTwist(x, y, heading, velocity);


        return chassisPose.createFactoredTwist(1/ticksPerInch);
    }

    @Override
    public void reset() {
        chassisPose.setTwist(new double[]{0, 0, 0, 0});
    }

    @Override
    public void reset(double[] pose) {
        chassisPose.setTwist(pose);
    }



}