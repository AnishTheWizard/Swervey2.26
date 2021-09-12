// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs.Wrappers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class LimeLight {
    private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public static double getHorizontalOffset() {
        return limelight.getEntry("tx").getDouble(0.0);
    }
}