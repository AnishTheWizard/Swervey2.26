// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.libs.Wrappers.GenericMotor;
import frc.robot.libs.Wrappers.Gyro;
import frc.robot.libs.Wrappers.LimeLight;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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

  private PIDController limelightController;

  private boolean driveTrainLock= false;

  public static DriveTrain getInstance() {
    if(driveTrain == null)
      driveTrain = new DriveTrain();
    return driveTrain;
  }
  public DriveTrain() {
    this.drives = new GenericMotor[Constants.NUMBER_OF_MODULES];
    this.steers = new GenericMotor[Constants.NUMBER_OF_MODULES];
    this.encoders = new GenericEncoder[Constants.NUMBER_OF_MODULES];
    this.gyro = new Gyro(new TalonSRX(RobotMap.GYRO)); //TODO
    // this.gyro = new Gyro(RobotMap.GYRO);
    this.limelightController = new PIDController(Constants.LIMELIGHT_GAINS[0], Constants.LIMELIGHT_GAINS[1], Constants.LIMELIGHT_GAINS[2]);
    for(int i = 0; i < Constants.NUMBER_OF_MODULES; i++) {
      CANSparkMax drive = new CANSparkMax(RobotMap.DRIVE_MOTORS[i], MotorType.kBrushless);
      CANSparkMax steer = new CANSparkMax(RobotMap.STEER_MOTORS[i], MotorType.kBrushless);
      // TalonFX drive = new TalonFX(RobotMap.DRIVE_MOTORS[i]);
      // VictorSPX steer = new VictorSPX(RobotMap.STEER_MOTORS[i]);
      AnalogInput encoder = new AnalogInput(RobotMap.ENCODERS[i]);
      steer.setInverted(true);
      drives[i] = new GenericMotor(drive);
      steers[i] = new GenericMotor(steer);
      encoders[i] = new GenericEncoder(encoder, Constants.TICKS_PER_ROTATION, Constants.OVERFLOW_THRESHOLD, Constants.MODULE_OFFSETS[i]);
    }
    swerve = new Swerve(drives, steers, encoders, gyro, Constants.MODULE_POSITIONS, Constants.MODULE_GAINS, new double[]{Constants.STEER_GAINS_HIGH, Constants.STEER_GAINS_THRESHOLD}, new double[]{Constants.ROTATE_GAINS_HIGH, Constants.ROTATE_GAINIS_THRESHOLD, Constants.ROTATE_VELOCITY_THRESHOLD},Constants.NUMBER_OF_MODULES, Constants.PERCENT_SPEED, Constants.TICKS_PER_FOOT);
  }

  public void control(double x, double y, double rotate) {
    swerve.control(x, y, rotate);
  }

  public void toggleSpeed() {
    swerve.toggleSpeed();
  }

  public void toPose(double[] pose) {
    swerve.toPose(pose);
  }

  public void reset() {
    swerve.zeroGyro();
    swerve.reset();
  }

  public void lockDriveTrain() {
    this.driveTrainLock = true;
  }

  public void unlockDriveTrain() {
    this.driveTrainLock = false;
  }

  public PIDController getRotationPIDController() {
    return limelightController;
  }

  @Override
  public void periodic() {
    if(!driveTrainLock && Robot.state == Robot.Phase.TELEOP)
      control(RobotContainer.getInstance().getLeftJoyX(), RobotContainer.getInstance().getLeftJoyY(), -RobotContainer.getInstance().getRightJoyX());
    double top = SmartDashboard.getNumber("Top Speed", 0.5);
    SmartDashboard.putNumber("Top Speed", top);
    swerve.setTopSpeed(top);
    
    //Debug
    for(int i = 0; i < Constants.NUMBER_OF_MODULES; i++) {
      SmartDashboard.putNumber("Encoders " + i, swerve.getModuleRotationalPose(i));
      SmartDashboard.putNumber("PoseX " + i, swerve.getModulePose(i)[0]);
      SmartDashboard.putNumber("PoseY " + i, swerve.getModulePose(i)[1]);
      SmartDashboard.putNumber("vel " + i, swerve.getModuleVelocity(i));
      SmartDashboard.putNumber("driving pose " + i, swerve.getModuleDrivePose(i));
    }
    
    double[] pose = swerve.getPose();
    SmartDashboard.putNumber("X", RobotContainer.getInstance().getLeftJoyX());
    SmartDashboard.putNumber("y", RobotContainer.getInstance().getLeftJoyY());
    SmartDashboard.putNumber("xpose", pose[0]);
    SmartDashboard.putNumber("ypose", pose[1]);
    SmartDashboard.putNumber("angle", pose[2]);
    }
}
