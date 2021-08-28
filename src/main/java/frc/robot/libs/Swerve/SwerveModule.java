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

    public SwerveModule(GenericMotor drive, GenericMotor steer, GenericEncoder steercoder, PIDController steerController) {
        this.drive = drive;
        this.steer = steer;
        this.steercoder = steercoder;
        this.steerController = steerController;

    }


    public void set(double velocity, double targetAngle) {

        double err = getError(targetAngle, steercoder.getContinousPosition());

        if(err > Math.PI/2) {
            err -= Math.PI;
            velocity *= -1;
        }
        else if(err < -Math.PI/2) {
            err += Math.PI;
            velocity*=-1;
        }
        double rotateSpeed = steerController.calculate(err);
        drive.set(velocity);
        steer.set(rotateSpeed);
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
}
