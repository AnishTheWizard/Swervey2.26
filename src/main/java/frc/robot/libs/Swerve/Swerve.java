// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs.Swerve;

import frc.robot.libs.Wrappers.GenericMotor;
import frc.robot.libs.Wrappers.GenericEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.Wrappers.Gyro;
import frc.robot.libs.Swerve.SwerveModule;

/** Add your docs here. */
public class Swerve {

  private SwerveModule[] modules;
  private Gyro gyro;

  private PIDController driveController;
  private PIDController steerController;
  private PIDController rotateController;

  private double[] speeds;
  private double[] thetas;

  private double percentSpeed, topPercentSpeed;
  private double[] rotationAngles;
  private double[][] modulePoses;

  private double rotateGainsThreshold;
  private double rotateVelocityThreshold;
  private double rotateHighGain;
  private double rotateLowGain;
  private boolean isGyroAngleSet;
  private double gyroHold;

  private double x, y;
  private double ticksPerFeet;


  public Swerve(GenericMotor[] drives, GenericMotor[] steers, GenericEncoder[] encoders, Gyro gyro, double[][] modulePositions, double[] pidGains, double[] steerGainsHighAndThreshold, double[] rotateGainsHighAndThresholds, int numberOfModules, double percentSpeed, double ticksPerFeet) {
    modules = new SwerveModule[numberOfModules];
    speeds = new double[numberOfModules];
    thetas = new double[numberOfModules];
    this.rotationAngles = new double[numberOfModules];
    this.modulePoses = modulePositions;
    this.driveController = new PIDController(pidGains[0], pidGains[1], pidGains[2]);
    this.steerController = new PIDController(pidGains[3], pidGains[4], pidGains[5]);
    this.rotateController = new PIDController(pidGains[6], pidGains[7], pidGains[8]);
    this.rotateHighGain = rotateGainsHighAndThresholds[0];
    this.rotateGainsThreshold = rotateGainsHighAndThresholds[1];
    this.rotateVelocityThreshold = rotateGainsHighAndThresholds[2];
    this.rotateLowGain = rotateController.getP();
    this.percentSpeed = percentSpeed;
    this.ticksPerFeet = ticksPerFeet;

    this.topPercentSpeed = 0.5;
    this.gyro = gyro;
    this.x = 0;
    this.y = 0;

    for(int i = 0; i < numberOfModules; i++) {
        modules[i] = new SwerveModule(drives[i],
                                      steers[i],
                                      encoders[i],
                                      steerController,
                                      steerGainsHighAndThreshold);
        speeds[i] = 0;
        thetas[i] = 0;
    }

    for(int i = 0; i < rotationAngles.length; i++) {
      rotationAngles[i] = Math.atan2(modulePositions[i][1], modulePositions[i][0]) + Math.PI/2;
      SmartDashboard.putNumber("key  " + i, rotationAngles[i]);
      SmartDashboard.putNumber("values " + i, modulePositions[i][0]);
    }
      
  }

  public void control(double x, double y, double rotate) {
    if(Math.abs(rotate) < rotateVelocityThreshold){
      if(!isGyroAngleSet) {
        gyroHold = gyro.getYaw();
        isGyroAngleSet = true;
      }

      rotateController.setP(
        Math.hypot(x, y) < rotateGainsThreshold ? rotateHighGain : rotateLowGain
      );
      rotate = rotateController.calculate(gyro.getYaw(), gyroHold);
    }
    else {
      isGyroAngleSet = false;
    }
      for(int i = 0; i < modules.length; i++) {

        double rotateVectorX = rotate * Math.cos(rotationAngles[i] + gyro.getYaw());
        double rotateVectorY = rotate * Math.sin(rotationAngles[i] + gyro.getYaw());

        double targetVectorX = x + rotateVectorX;
        double targetVectorY = y + rotateVectorY;

        double theta = Math.atan2(targetVectorY, targetVectorX);
        double speed = Math.hypot(targetVectorY, targetVectorX);

        theta -= gyro.getYaw();

        if(!(x == 0 && y == 0 && rotate == 0)) {
          thetas[i] = theta;
        }
        speeds[i] = speed;
      }

      speeds = normalize(speeds);
      SmartDashboard.putNumber("speed ", speeds[0]);
      SmartDashboard.putNumber("thetea ", thetas[0]);

      for(int i = 0; i < modules.length; i++) {
        modules[i].set(speeds[i] * percentSpeed, thetas[i], gyro.getYaw());
        SmartDashboard.putNumber("speeds" + i, speeds[i] * percentSpeed);
      }
  }

  public void toggleSpeed() {
    if(percentSpeed == topPercentSpeed) {
      percentSpeed = 0.3;
    }
    else {
      percentSpeed = topPercentSpeed;
    }
  }

  public void setTopSpeed(double top) {
    this.topPercentSpeed = top;
  }

  private double[] normalize(double[] arr) {
    double maxVal = 1;
    for(int i = 0; i < arr.length; i++) {
      if(arr[i] > maxVal) {
        maxVal = arr[i];
      }
    }

    for(int i = 0; i < arr.length; i++) {
      arr[i] /= maxVal;
    }

    return arr;
  }

  public double getModuleRotationalPose(int module) {
    return modules[module].getModuleRotationalPose();
  }

  public double[] getPose() {
    x = 0;
    y = 0;
    for(int i = 0; i < modules.length; i++) {
      x += modules[i].getCurrentModulePosition()[0]/modules.length;
      y += modules[i].getCurrentModulePosition()[1]/modules.length;
    }
    return new double[]{x/ticksPerFeet, y/ticksPerFeet, gyro.getYaw()};
  }

  public double[] getModulePose(int i) {
    return new double[] {modules[i].getCurrentModulePosition()[0]/ticksPerFeet, modules[i].getCurrentModulePosition()[1]/ticksPerFeet};
  }

  public double getModuleDrivePose(int i) {
    return modules[i].getDrivePose();
  }

  public double getModuleVelocity(int i) {
    return modules[i].getDriveVelocity();
  }

  public void toPose(double[] target) {
    //TODO add to position function
    // find err
    
  }

  public void zeroGyro() {
    gyro.zeroGyro();
  }

  public void reset() {
    for(int i = 0; i < modules.length; i++) {
      modules[i].reset();
    }

    for(int i = 0; i < modules.length; i++) {
      modules[i].setPose(modulePoses[i][0] * ticksPerFeet, modulePoses[i][1] * ticksPerFeet);
    }
  }
}
