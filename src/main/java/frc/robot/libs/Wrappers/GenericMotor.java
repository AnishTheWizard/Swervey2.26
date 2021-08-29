// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs.Wrappers;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants.PassiveMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

/** Add your docs here.
 * @author Anish Chandra
 */
public class GenericMotor {
    //supports falcons, talons, sparks, victors
    private TalonFX falcon;
    private CANSparkMax spark;
    private TalonSRX talon;
    private VictorSPX victor;

    private enum MotorType {
        FALCON,
        SPARK,
        TALON,
        VICTOR
    }

    private MotorType motorType;

    private double lastSpeed;
    private double lastSensorPose;

    public GenericMotor(TalonFX falcon) {
        this.falcon = falcon;
        motorType = MotorType.FALCON;
        this.lastSpeed = 0;
        this.lastSensorPose = 0;
    }

    public GenericMotor(CANSparkMax spark) {
        this.spark = spark;
        motorType = MotorType.SPARK;
        this.lastSpeed = 0;
        this.lastSensorPose = 0;
    }

    public GenericMotor(TalonSRX talon) {
        this.talon = talon;
        motorType = MotorType.TALON;
        this.lastSpeed = 0;
        this.lastSensorPose = 0;
    }

    public GenericMotor(VictorSPX victor) {
        this.victor = victor;
        motorType = MotorType.VICTOR;
        this.lastSpeed = 0;
        this.lastSensorPose = 0;
    }

    public void set(double speed) {
        if(speed != lastSpeed) {
            switch(motorType) {
                case FALCON:
                    falcon.set(ControlMode.PercentOutput, speed);
                    break;
                case SPARK:
                    spark.set(speed);
                    break;
                case TALON:
                    talon.set(ControlMode.PercentOutput, speed);
                    break;
                case VICTOR:
                    victor.set(ControlMode.PercentOutput, speed);
                    break;
                default:
                    break;

            }
            lastSpeed = speed;
        }
    }

    public double getSensorPose() {
        switch(motorType) {
            case FALCON:
                return falcon.getSelectedSensorPosition();
            case TALON:
                return talon.getSelectedSensorPosition();
            case VICTOR:
                return victor.getSelectedSensorPosition();
            default:
                return -1;

        }
    }

    public double getSensorErr() {
        double err;
        switch(motorType) {
            case FALCON:
                err = falcon.getSelectedSensorPosition() - lastSensorPose;
                lastSensorPose = falcon.getSelectedSensorPosition();
            case TALON:
                err = talon.getSelectedSensorPosition() - lastSensorPose;
                lastSensorPose = talon.getSelectedSensorPosition();
            case VICTOR:
                err = victor.getSelectedSensorPosition() - lastSensorPose;
                lastSensorPose = victor.getSelectedSensorPosition();
            default:
                err = -1;
        }
        return err;
    }

    public void inverted(boolean invert) {
        switch(motorType) {
            case FALCON:
                falcon.setInverted(invert);
                break;
            case SPARK:
                spark.setInverted(invert);
                break;
            case TALON:
                talon.setInverted(invert);
                break;
            case VICTOR:
                victor.setInverted(invert);
                break;
            default:
                break;
        }
    }

    public boolean setSensorPose(double sensorPos) {
        switch(motorType) {
            case FALCON:
                falcon.setSelectedSensorPosition(sensorPos);
                break;
            case TALON:
                talon.setSelectedSensorPosition(sensorPos);
                break;
            case VICTOR:
                victor.setSelectedSensorPosition(sensorPos);
                break;
            default:
                return false;
        }
        return true;
    }

    public void setNeutralMode(PassiveMode neutralMode) {
        switch(motorType) {
            case FALCON:
                if(neutralMode.equals(PassiveMode.BRAKE)) {
                    falcon.setNeutralMode(NeutralMode.Brake);
                }
                else if(neutralMode.equals(PassiveMode.COAST)) {
                    falcon.setNeutralMode(NeutralMode.Coast);
                }
                break;
            case SPARK:
                if(neutralMode.equals(PassiveMode.BRAKE)) {
                    spark.setIdleMode(IdleMode.kBrake);
                }
                else if(neutralMode.equals(PassiveMode.COAST)) {
                    spark.setIdleMode(IdleMode.kCoast);
                }
                break;
            case TALON:
                if(neutralMode.equals(PassiveMode.BRAKE)) {
                    talon.setNeutralMode(NeutralMode.Brake);
                }
                else if(neutralMode.equals(PassiveMode.COAST)) {
                    talon.setNeutralMode(NeutralMode.Coast);
                }
                break;
            case VICTOR:
                if(neutralMode.equals(PassiveMode.BRAKE)) {
                    victor.setNeutralMode(NeutralMode.Brake);
                }
                else if(neutralMode.equals(PassiveMode.COAST)) {
                    victor.setNeutralMode(NeutralMode.Coast);
                }
                break;
            default:
                break;

        }
    }

    public void configFalcon(TalonFXConfiguration config) {
        falcon.configAllSettings(config);
    }

    public void configTalon(TalonSRXConfiguration config) {
        talon.configAllSettings(config);
    }

    public void configVictor(VictorSPXConfiguration config) {
        victor.configAllSettings(config);
    }
}
