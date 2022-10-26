// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.libs.swervey;

import frc.libs.wrappers.GenericMotor;
import frc.libs.wrappers.GenericEncoder;
import frc.libs.wrappers.Gyro;

import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * @author Anish Chandra
 * Handles the Swerve DriveTrain
*/
public class Swerve {

  private SwerveModule[] modules;
  private Gyro gyro;

  private PIDController driveController;
  private PIDController steerController;
  private PIDController rotateController;

  private double[] speeds;
  private double[] thetas;

  private double lowPercentSpeed, topPercentSpeed, currentPercentSpeed;
  private double[] rotationAngles;
  private double[][] modulePoses;

  private double rotateGainsThreshold;
  private double rotateVelocityThreshold;
  private double[] rotateHighGains;
  private double[] rotateLowGains;
  private boolean isGyroAngleSet;
  private double gyroHold;

  private double x, y;
  private double ticksPerFeet;
  private double[] target;
  private double allowedTranslationalError;
  private double allowedRotationalError;


  public Swerve(GenericMotor[] drives, GenericMotor[] steers, GenericEncoder[] encoders, Gyro gyro, double[][] modulePositions, int numberOfModules) {
    modules = new SwerveModule[numberOfModules];
    speeds = new double[numberOfModules];
    thetas = new double[numberOfModules];
    this.rotationAngles = new double[numberOfModules];
    this.modulePoses = modulePositions;
    this.driveController = new PIDController(0, 0, 0);
    this.steerController = new PIDController(0, 0, 0);
    this.rotateController = new PIDController(0, 0, 0);

    this.gyro = gyro;
    this.x = 0;
    this.y = 0;
    this.target = new double[3];

    for(int i = 0; i < numberOfModules; i++) {
        modules[i] = new SwerveModule(drives[i],
                                      steers[i],
                                      encoders[i],
                                      steerController,
                                      modulePositions[i]);
        speeds[i] = 0;
        thetas[i] = 0;
    }

    for(int i = 0; i < rotationAngles.length; i++) 
      rotationAngles[i] = Math.atan2(modulePositions[i][1], modulePositions[i][0]) + Math.PI/2;
  }

  public void configureSteerPIDGains(double[] highGains, double[] lowGains) {
    for(SwerveModule mod : modules) mod.configureSteerPIDGains(highGains, lowGains);
  }

  public void configureDrivePIDGains(double[] gains) {
    this.driveController.setPID(gains[0], gains[1], gains[2]);
  }

  public void configureRotatePIDGains(double[] highGains, double[] lowGains) {
    this.rotateHighGains = highGains;
    this.rotateLowGains = lowGains;
    this.rotateController.setPID(rotateLowGains[0], rotateLowGains[1], rotateLowGains[1]);
  }

  public void configureThresholds(double sThresh, double rThresh, double rVelThresh) {
    for(SwerveModule mod : modules) mod.configureSteerThreshold(sThresh);
    this.rotateGainsThreshold = rThresh;
    this.rotateVelocityThreshold = rVelThresh;
  }

  public void configureAutonomousParameters(double ticksPerFeet, double translationalError, double rotationalError) {
    this.ticksPerFeet = ticksPerFeet;
    this.allowedTranslationalError = translationalError;
    this.allowedRotationalError = rotationalError;
  }

  public void configureSpeeds(double topSpeed, double lowSpeed) {
    this.topPercentSpeed = topSpeed;
    this.lowPercentSpeed = lowSpeed;
  }

  public void control(double x, double y, double rotate) {
    if(Math.abs(rotate) < rotateVelocityThreshold){
      if(!isGyroAngleSet) {
        gyroHold = gyro.getYaw();
        isGyroAngleSet = true;
      }

      if(Math.hypot(x, y) < rotateGainsThreshold) rotateController.setPID(rotateHighGains[0], rotateHighGains[1], rotateHighGains[2]);
      else rotateController.setPID(rotateLowGains[0], rotateLowGains[1], rotateLowGains[2]);

      rotate = rotateController.calculate(gyro.getYaw(), gyroHold);
      if(Math.abs(rotate) < rotateVelocityThreshold) rotate = 0;

      
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

      for(int i = 0; i < modules.length; i++) {
        modules[i].set(speeds[i] * lowPercentSpeed, thetas[i], gyro.getYaw());
      }
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

  public void toggleSpeed() {
    if(currentPercentSpeed == topPercentSpeed) {
      currentPercentSpeed = lowPercentSpeed;
    }
    else {
      currentPercentSpeed = topPercentSpeed;
    }
  }

  public void setTopSpeed(double top) {
    this.topPercentSpeed = top;
  }

  public void setTargetPosition(double[] target) {
    this.target = target;
  }

  public double getCurrentSpeedMultiplier() {
    return currentPercentSpeed;
  }

  public double getModuleRotationalPose(int module) {
    return modules[module].getModuleRotationalPose();
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

  public double[] getPose() {
    x = 0;
    y = 0;
    for(int i = 0; i < modules.length; i++) {
      x += modules[i].getCurrentModulePosition()[0]/modules.length;
      y += modules[i].getCurrentModulePosition()[1]/modules.length;
    }
    return new double[]{x/ticksPerFeet, y/ticksPerFeet, gyro.getYaw()};
  }
  
  public void toPose(double[] target) {
    double[] currentPose = getPose();
    this.target = target;
    double xErr = driveController.calculate(currentPose[0], target[0]);
    double yErr = driveController.calculate(currentPose[1], target[1]);
    double speed = Math.hypot(xErr, yErr);

    if(speed < rotateGainsThreshold) rotateController.setPID(rotateHighGains[0], rotateHighGains[1], rotateHighGains[2]);
    else rotateController.setPID(rotateLowGains[0], rotateLowGains[1], rotateLowGains[2]);

    double rotateErr = rotateController.calculate(currentPose[2], target[2]);

    control(xErr, yErr, rotateErr);
  }

  public boolean atSetpoint() {
    double[] currentPose = getPose();
    int correctPoints = 0;
    for(int i = 0; i < currentPose.length; i++) {
      if(currentPose[i] < (target[i] + (i != 2 ? allowedTranslationalError : allowedRotationalError)) && currentPose[i] > target[i] - (i != 2 ? allowedTranslationalError : allowedRotationalError)) {
        correctPoints++;
      }
    }
    return correctPoints == currentPose.length;
  }

  public void zeroGyro() {
    gyro.zeroGyro();
    isGyroAngleSet = false;
  }

  public void reset() {
    for(int i = 0; i < modules.length; i++) {
      modules[i].setPose((modulePoses[i][0] * ticksPerFeet), (modulePoses[i][1] * ticksPerFeet));
    }
  }
}
