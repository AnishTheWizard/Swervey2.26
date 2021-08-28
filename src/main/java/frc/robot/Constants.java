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
    public static enum PassiveMode {
        BRAKE,
        COAST
    }

    public static final int OVERFLOW_THRESHOLD = 2048;
    // public static final double OVERFLOW_THRESHOLD = Math.PI; // for cancoders

    public static final int[] MODULE_OFFSETS = {-2010, -3070, -1274, -3179}; //{2109, 3035, 1298, 3113};

    public static final int TICKS_PER_ROTATION = 4096;

    public static final int NUMBER_OF_MODULES = 4;

    public static final double[] MODULE_POSITIONS = {0, 0, 0, 0};

    public static final double[] STEER_GAINS = {0.226, 0, 0};

    public static final double CONTROLLER_DEADBAND = 0.2;

    public static final double PERCENT_SPEED = 0.4;
}
