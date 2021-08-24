// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs.Wrappers;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

/** Add your docs here. 
 * @author Anish Chandra
*/
public class Gyro {
    private PigeonIMU pigeon;
    //TODO take a look at absolute compass mode

    public Gyro(TalonSRX controller) {
        pigeon = new PigeonIMU(controller);
    }

    public Gyro(int port) {
        pigeon = new PigeonIMU(port);
    }

    public double getYaw() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return Math.toRadians(ypr[0]);
    }

    public double getAbsoluteDirection() {
        return pigeon.getAbsoluteCompassHeading();
    }

    public void zeroGyro() {
        pigeon.setYaw(0);
    }

}
