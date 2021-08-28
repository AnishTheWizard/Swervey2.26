// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.libs.Wrappers.GenericMotor;
import frc.robot.libs.Wrappers.Gyro;
import edu.wpi.first.wpilibj.AnalogInput;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.Swerve.Swerve;
import frc.robot.libs.Wrappers.GenericEncoder;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  //TESTING CODE
  private Swerve swerve;


  private static DriveTrain driveTrain = null;
  private GenericMotor[] drives;
  private GenericMotor[] steers;
  private GenericEncoder[] encoders;
  private Gyro gyro;

  public static DriveTrain getInstance() {
    if(driveTrain == null)
      driveTrain = new DriveTrain();
    return driveTrain;
  }
  public DriveTrain() {
    this.drives = new GenericMotor[Constants.NUMBER_OF_MODULES];
    this.steers = new GenericMotor[Constants.NUMBER_OF_MODULES];
    this.encoders = new GenericEncoder[Constants.NUMBER_OF_MODULES];
    this.gyro = new Gyro(new TalonSRX(RobotMap.GYRO));
    for(int i = 0; i < Constants.NUMBER_OF_MODULES; i++) {
      CANSparkMax drive = new CANSparkMax(RobotMap.DRIVE_MOTORS[i], MotorType.kBrushless);
      CANSparkMax steer = new CANSparkMax(RobotMap.STEER_MOTORS[i], MotorType.kBrushless);
      AnalogInput encoder = new AnalogInput(RobotMap.ENCODERS[i]);
      steer.setInverted(true);
      drives[i] = new GenericMotor(drive);
      steers[i] = new GenericMotor(steer);
      encoders[i] = new GenericEncoder(encoder, Constants.TICKS_PER_ROTATION, Constants.OVERFLOW_THRESHOLD, Constants.MODULE_OFFSETS[i]);
    }
    swerve = new Swerve(drives, steers, encoders, gyro, Constants.MODULE_POSITIONS, Constants.STEER_GAINS, Constants.NUMBER_OF_MODULES, Constants.PERCENT_SPEED);
  }

  public void reset() {
    swerve.zeroGyro();
  }


  @Override
  public void periodic() {
    swerve.control(RobotContainer.getInstance().getLeftJoyX(), RobotContainer.getInstance().getLeftJoyY(), -RobotContainer.getInstance().getRightJoyX());
    
    //Debug
    for(int i = 0; i < Constants.NUMBER_OF_MODULES; i++)
      SmartDashboard.putNumber("Encoders " + i, swerve.getModuleRotationalPose(i));
    SmartDashboard.putNumber("X", RobotContainer.getInstance().getLeftJoyX());
    SmartDashboard.putNumber("y", RobotContainer.getInstance().getLeftJoyY());
    }
}
