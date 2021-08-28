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


  private double[] speeds;
  private double[] thetas;

  private double percentSpeed;



  public Swerve(GenericMotor[] drives, GenericMotor[] steers, GenericEncoder[] encoders, Gyro gyro, double[] modulePositions, double[] pidGains, int numberOfModules, double percentSpeed) {
    modules = new SwerveModule[numberOfModules];
    speeds = new double[numberOfModules];
    thetas = new double[numberOfModules];
    this.percentSpeed = percentSpeed; 
    this.gyro = gyro;
    for(int i = 0; i < numberOfModules; i++) {
        modules[i] = new SwerveModule(drives[i],
                                      steers[i],
                                      encoders[i],
                                      new PIDController(pidGains[0],
                                                        pidGains[1],
                                                        pidGains[2])
                                      );
        speeds[i] = 0;
        thetas[i] = 0;
    }
      
  }

  public void control(double x, double y, double rotate) {
      for(int i = 0; i < modules.length; i++) {
        double theta = Math.atan2(y, x);
        double speed = Math.hypot(y, x);
        
        theta -= gyro.getYaw();

        if(!(x == 0 && y == 0 && rotate == 0)) {
          thetas[i] = theta;
        }
        speeds[i] = speed;
      }

      speeds = normalize(speeds);
      SmartDashboard.putNumber("speed ", speeds[0]);
      SmartDashboard.putNumber("thetea ", thetas[0]);

      for(int i = 0; i < modules.length; i++) 
        modules[i].set(speeds[i] * percentSpeed, thetas[i]);
        // SmartDashboard.putNumber("speeds" + i, speeds[i]);
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

  public void zeroGyro() {
    gyro.zeroGyro();
  }
}
