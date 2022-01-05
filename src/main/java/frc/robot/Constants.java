// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Swerve Configuration Parameters
 

    // Encoder Parameters
    public static final int OVERFLOW_THRESHOLD = 2048;

    public static final int[] MODULE_OFFSETS = {-2010, -3070, -1274, -3179};

    public static final int TICKS_PER_ROTATION = 4096;


    // Physical Configuration 
    public static final int NUMBER_OF_MODULES = 4;

    public static final double LENGTH = 23.5;
    public static final double WIDTH = 21.5;


    // Controller Configuration
    public static final double CONTROLLER_DEADBAND = 0.1;

    
    // Autonomous Parameters
    public static final double TRANSLATAIONAL_ERROR = 0.4;
    public static final double ROTATE_ERROR = 0.25;

    public static final double TICKS_PER_FOOT = 274.2227618497799;


    // Teleoperated Parameters
    public static final double LOW_BOUND_SPEED = 0.3;
    public static final double HIGH_BOUND_SPEED = 0.5;


    // PID Configurations
    public static final double[] DRIVE_GAINS = {0.503, 0.0, 0.0};

    public static final double[] STEER_GAINS_LOW = {0.226, 0.0, 0.0};
    public static final double[] STEER_GAINS_HIGH = {0.35, 0.0, 0.0};

    public static final double[] ROTATE_GAINS_LOW = {0.669, 0.0, 0.0};
    public static final double[] ROTATE_GAINS_HIGH = {1.0, 0.0, 0.0};

    public static final double[] LIMELIGHT_GAINS = {0.0226, 0.0, 0.0};

	public static final double STEER_GAINS_THRESHOLD = 0.08;
    public static final double ROTATE_GAINS_THRESHOLD = 0.08;
    public static final double ROTATE_VELOCITY_THRESHOLD = 0.09;


    // REFORMATTED SWERVE PARAMETERS
    public static final double[][] MODULE_GAINS = {DRIVE_GAINS,
                                                   STEER_GAINS_LOW,
                                                   ROTATE_GAINS_LOW};
                                                 
    public static final double[][] SCHEDULED_GAINS = {STEER_GAINS_HIGH,
                                                      ROTATE_GAINS_HIGH};

    public static final double[] STEER_AND_ROTATE_THRESHOLDS = {STEER_GAINS_THRESHOLD, ROTATE_GAINS_THRESHOLD, ROTATE_VELOCITY_THRESHOLD};

    public static final double[][] MODULE_POSITIONS =  {new double[]{WIDTH/2, LENGTH/2}, 
                                                        new double[]{-WIDTH/2, LENGTH/2}, 
                                                        new double[]{-WIDTH/2, -LENGTH/2}, 
                                                        new double[]{WIDTH/2, -LENGTH/2}};

    public static final double[] ALLOWED_ERRORS = {TRANSLATAIONAL_ERROR, ROTATE_ERROR};

    public static final double[] SPEED_BOUNDS = {LOW_BOUND_SPEED, HIGH_BOUND_SPEED};

}
