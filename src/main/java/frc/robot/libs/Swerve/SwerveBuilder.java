// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs.Swerve;

import frc.robot.libs.Wrappers.GenericMotor;
import frc.robot.libs.Wrappers.GenericEncoder;
import frc.robot.libs.Wrappers.Gyro;

/** 
 * @author Anish Chandra
 * Builds the Swerve Object
*/
public class SwerveBuilder {
    private GenericMotor[] drives;
    private GenericMotor[] steers;
    private GenericEncoder[] encoders;
    private Gyro gyro;

    private double[][] modulePositions;
    private double[][] pidGains;
    private double[][] scheduledGains;
    private double[] thresholds;

    int numberOfModules;

    double[] speedBounds;

    double ticksPerFeet;
    double[] allowedErrors;

    boolean[] verfiedParts;

    public SwerveBuilder(GenericMotor[] drives, GenericMotor[] steers, GenericEncoder[] encoders, Gyro gyro) {
        this.drives = drives;
        this.steers = steers;
        this.encoders = encoders;
        this.gyro = gyro;
    }

    public Swerve buildSwerve() throws NullPointerException {
        for(boolean boo : verfiedParts) {
            if(!boo) {
                throw new NullPointerException("Missing Swerve Parameters");
            }
        }

        Swerve swerve = new Swerve(drives, steers, encoders, gyro, modulePositions, numberOfModules);
        swerve.configureDrivePIDGains(pidGains[0]);
        swerve.configureSteerPIDGains(scheduledGains[0], pidGains[1]);
        swerve.configureRotatePIDGains(scheduledGains[1], pidGains[2]);

        swerve.configureThresholds(thresholds[0], thresholds[1], thresholds[2]);

        swerve.configureAutonomousParameters(ticksPerFeet, allowedErrors[0], allowedErrors[1]);

        swerve.configureSpeeds(speedBounds[0], speedBounds[1]);

        return swerve;
    }

    public SwerveBuilder PIDGains(double[][] pidGains, double[][] scheduledGains, double[] thresholds) {
        this.pidGains = pidGains;
        this.scheduledGains = scheduledGains;
        this.thresholds = thresholds;
        verfiedParts[0] = true;
        return this;
    }

    public SwerveBuilder modulePositions(double[][] modPoses) {
        this.modulePositions = modPoses;
        verfiedParts[1] = true;
        return this;
    }

    public SwerveBuilder speedBounds(double[] speedBounds) {
        this.speedBounds = speedBounds;
        verfiedParts[2] = true;
        return this;
    }

    public SwerveBuilder autonomousParameters(double ticksPerFeet, double[] allowedErrors) {
        this.ticksPerFeet = ticksPerFeet;
        this.allowedErrors = allowedErrors;
        verfiedParts[3] = true;
        return this;
    }    
}
