// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.libs.wrappers.Controller;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private DriveTrainOld driveTrainOld;

  private DriveTrain dt;

  private final Controller joy = new Controller(0, .25);
 
  private static RobotContainer robotContainer = null;

  public static RobotContainer getInstance() {
    if(robotContainer == null)
      robotContainer = new RobotContainer();
    return robotContainer;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    dt = DriveTrain.getInstance();

    dt.setDefaultCommand(new RunCommand(() -> dt.control(getLeftJoyX(), getLeftJoyY(), getRightJoyX()), dt));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    joy.getSTARTButton().onTrue(new InstantCommand(() -> dt.reset()));
    // joy.getBButton().whenHeld(new RunCommand(() -> driveTrain.toPose(new double[]{0, 0, 0}), DriveTrain.getInstance()));
    // joy.getYButton().whenPressed(() -> driveTrainOld.toggleSpeed());
    // joy.getXButton().whenPressed(() -> driveTrain.control(0.0, 0.3, 0.0));
    // joy.getBButton().whenPressed(new MoveForward());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand(() -> System.out.println("autonomous"));
  }


  public double getLeftJoyX() {
    return joy.getLeftJoyX();
  }

  public double getRightJoyX() {
    return joy.getRightJoyX();
  }

  public double getLeftJoyY() {
    return joy.getLeftJoyY();
  }

  public double getRightJoyY() {
    return joy.getRightJoyY();
  }
}
