package frc.libs.swervey;

import edu.wpi.first.wpilibj.geometry.Twist2d;

public class SwerveOdometry {

    private SwerveModule[] modules;
    private double[] lastPositions;

    public SwerveOdometry(SwerveModule[] modules, double[] lastPositions) {
        this.modules = modules;
        this.lastPositions = lastPositions;
    }

    // threaded function that will run in the background constantly trying to update the position of the swerve drive
    public void updatePosition() {

    }
}
