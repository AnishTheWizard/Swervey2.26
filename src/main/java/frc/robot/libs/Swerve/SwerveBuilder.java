// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs.Swerve;

import frc.robot.libs.Wrappers.Gyro;
import frc.robot.libs.Wrappers.GenericEncoder;
import frc.robot.libs.Wrappers.GenericMotor;

/** 
 * @author Anish Chandra
 * Used to clear up and handle numerous parameters
*/
public class SwerveBuilder {
    // GenericMotor[] drives, GenericMotor[] steers, GenericEncoder[] encoders, Gyro gyro, double[][] modulePositions, double[] pidGains, double[] steerGainsHighAndThreshold, double[] rotateGainsHighAndThresholds, int numberOfModules, double percentSpeed, double ticksPerFeet, double allowedTranslationalError, double allowedRotationalError
    private GenericMotor[] drives;
    private GenericMotor[] steers;
    private GenericEncoder[] encoders;
    private Gyro gyro;

    private double[][] modulePositions;
    private double[][] pidGains;
    private double[][] scheduledGains;

    int numberOfModules;

    double[] speedBounds;

    double ticksPerFeet;
    double[] allowedErrors;

    SwerveBuilder(GenericMotor[] drives, GenericMotor[] steers, GenericEncoder[] encoders, Gyro gyro) {
        this.drives = drives;
        this.steers = steers;
        this.encoders = encoders;
        this.gyro = gyro;
    }

    Swerve buildSwerve() {
        return new Swerve()
    }

    SwerveBuilder PIDGains(double[][] pidGains, double[][] scheduledGains) {
        this.pidGains = pidGains;
        this.scheduledGains = scheduledGains;
        return this;
    }

    SwerveBuilder modulePositions(double[][] modPoses) {
        this.modulePositions = modPoses;
        return this;
    }

    SwerveBuilder speedBounds(double[] speedBounds) {
        this.speedBounds = speedBounds;
        return this;
    }

    SwerveBuilder autonomousParameters(double ticksPerFeet, double[] allowedErrors) {
        this.ticksPerFeet = ticksPerFeet;
        this.allowedErrors = allowedErrors;
        return this;
    }


    
}
