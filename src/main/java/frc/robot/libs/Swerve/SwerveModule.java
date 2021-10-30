// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs.Swerve;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.Wrappers.GenericEncoder;
import frc.robot.libs.Wrappers.GenericMotor;

/** Add your docs here. 
 * @author Anish Chandra
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
    private double lastSensorPose;

    // public SwerveModule(GenericMotor drive, GenericMotor steer, GenericEncoder steercoder, PIDController steerController, double[] steerGainsHighAndThreshold) {
    //     this.drive = drive;
    //     this.steer = steer;
    //     this.steercoder = steercoder;
    //     this.steerController = steerController;
    //     this.steerHighGain = steerGainsHighAndThreshold[0];
    //     this.steerLowGain = steerController.getP();
    //     this.threshold = steerGainsHighAndThreshold[1];
    //     this.x = 0;
    //     this.y = 0;//TODO PROB NOT CORRECT

    // }

    public SwerveModule(GenericMotor drive, GenericMotor steer, GenericEncoder steercoder, PIDController steerController, double[] modulePosition) {
        this.drive = drive;
        this.steer = steer;
        this.steercoder = steercoder;
        this.steerController = steerController;
        this.x = modulePosition[0];
        this.y = modulePosition[1];
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

        //split into vector components
        double ang = steercoder.getContinousPosition() % (2 * Math.PI);
        double mag = drive.getSensorPose() - lastSensorPose;

        x += mag * Math.cos(ang + gyroAngle);
        y += mag * Math.sin(ang + gyroAngle);

        lastSensorPose = drive.getSensorPose();

        // steerController.setP(
        //     velocity < threshold ? steerHighGain : steerLowGain
        // ); // if vel is less than thresh, then you need more force to spin the wheel

        if(velocity < threshold) steerController.setPID(steerHighGains[0], steerHighGains[1], steerHighGains[2]);
        else steerController.setPID(steerLowGains[0], steerLowGains[1], steerLowGains[2]);

        double rotateSpeed = steerController.calculate(err);
        drive.set(velocity);
        steer.set(rotateSpeed);
        SmartDashboard.putNumber("steerspeed of mod 4 probably ", rotateSpeed);
        SmartDashboard.putNumber("err be like ", err);
    }

    private double getError(double target, double current) {
        double err = (target - current) % (2 * Math.PI); //finds the error and brings it down to 0-2pi
        
        if(err > Math.PI) err -=  2 * Math.PI;
        else if(err < -Math.PI) err += 2 * Math.PI; //negative errors are treated differently, to counter the module not going above 2092 (mod offset)
        
        return err;
    }

    public double getModuleRotationalPose() {
        return steercoder.getCPTicks();
    }

    public double[] getCurrentModulePosition() {
        return new double[]{x, y}; 
    }

    public void setPose(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getDrivePose() {
        return drive.getSensorPose();
    }

    public double getDriveVelocity() {
        return drive.getVelocity();
    }

    public void reset() {
        x = 0;
        y = 0;
    }
}
