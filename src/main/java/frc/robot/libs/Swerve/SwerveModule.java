// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs.Swerve;

import frc.robot.libs.Wrappers.GenericMotor;
import frc.robot.libs.Wrappers.GenericEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;

/** 
 * @author Anish Chandra
 * Manages all components within a Swerve Module
*/
public class SwerveModule {

    private GenericMotor drive;
    private GenericMotor steer;
    private GenericEncoder steercoder;

    private PIDController steerController;
    private double[] steerHighGains;
    private double[] steerLowGains;
    private double threshold;

    private double x, y;
    private final double[] defaultModulePosition;
    private double lastSensorPose;

    public SwerveModule(GenericMotor drive, GenericMotor steer, GenericEncoder steercoder, PIDController steerController, double[] modulePosition) {
        this.drive = drive;
        this.steer = steer;
        this.steercoder = steercoder;
        this.steerController = steerController;
        this.defaultModulePosition = modulePosition;
        this.x = modulePosition[0];
        this.y = modulePosition[1];
        this.lastSensorPose = 0;
    }

    public void configureSteerPIDGains(double[] highGains, double[] lowGains) {
        this.steerHighGains = highGains;
        this.steerLowGains = lowGains;
        this.steerController.setPID(lowGains[0], lowGains[1], lowGains[2]);
    }

    public void configureSteerThreshold(double sThresh) {
        this.threshold = sThresh;
    }

    public void set(double velocity, double targetAngle, double gyroAngle) {

        double err = getError(targetAngle, steercoder.getContinousPosition());

        if(err > Math.PI/2) {
            err -= Math.PI;
            velocity *= -1;
        }
        else if(err < -Math.PI/2) {
            err += Math.PI;
            velocity*=-1;
        }

        double direction = steercoder.getContinousPosition() % (2 * Math.PI);
        double distance = drive.getSensorPose() - lastSensorPose;

        x += distance * Math.cos(direction + gyroAngle);
        y += distance * Math.sin(direction + gyroAngle);

        lastSensorPose = drive.getSensorPose();

        if(velocity < threshold) steerController.setPID(steerHighGains[0], steerHighGains[1], steerHighGains[2]);
        else steerController.setPID(steerLowGains[0], steerLowGains[1], steerLowGains[2]);

        double rotateSpeed = steerController.calculate(err);
        drive.set(velocity);
        steer.set(rotateSpeed);
    }

    private double getError(double target, double current) {
        double err = (target - current) % (2 * Math.PI);
        
        if(err > Math.PI) err -=  2 * Math.PI;
        else if(err < -Math.PI) err += 2 * Math.PI;
        
        return err;
    }

    public double getModuleRotationalPose() {
        return steercoder.getCPTicks();
    }

    public double getDrivePose() {
        return drive.getSensorPose();
    }

    public double getDriveVelocity() {
        return drive.getVelocity();
    }

    public double[] getCurrentModulePosition() {
        return new double[]{x, y}; 
    }

    public void setPose(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void reset() {
        x = defaultModulePosition[0];
        y = defaultModulePosition[1];
    }
}
