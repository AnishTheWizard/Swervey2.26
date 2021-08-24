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

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.Wrappers.GenericEncoder;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  //TESTING CODE
  private GenericMotor drive0 = new GenericMotor(new CANSparkMax(RobotMap.DRIVE_MOTORS[0], MotorType.kBrushless));
  private GenericMotor steer0 = new GenericMotor(new CANSparkMax(RobotMap.STEER_MOTORS[0], MotorType.kBrushless));
  private GenericEncoder encoder0 = new GenericEncoder(new AnalogInput(RobotMap.ENCODERS[0]), Constants.TICKS_PER_ROTATION, Constants.OVERFLOW_THRESHOLD, Constants.MODULE_OFFSETS[0]);
  private Gyro gyro = new Gyro(new TalonSRX(RobotMap.GYRO));


  private static DriveTrain driveTrain = null;

  public static DriveTrain getInstance() {
    if(driveTrain == null)
      driveTrain = new DriveTrain();
    return driveTrain;
  }
  public DriveTrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("actual gyro", gyro.getYaw());
    SmartDashboard.putNumber("encoder", encoder0.getContinousPosition());
    SmartDashboard.putNumber("tick", encoder0.getCPTicks());

    steer0.set(RobotContainer.getInstance().joy.getRightJoyX() * 0.2);
    drive0.set(RobotContainer.getInstance().joy.getLeftJoyY() * 0.2);
  }
}
