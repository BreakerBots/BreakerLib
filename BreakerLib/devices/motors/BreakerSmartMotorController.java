// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/** BreakerLib smart motor controller for REV and CTRE motors. */
public class BreakerSmartMotorController implements MotorController {
    
    private ControllerType controllerType;
    private CANSparkMax sparkMax;
    private BaseMotorController ctreController;
    private double speed = 0.0;
    private boolean isInverted = false;
    private int pidSlot;

    private enum ControllerType {
        CTRE,
        SPARK_MAX
    }

    public enum SmartMotorControlMode {
        VOLTAGE,
        PRECENT,
        POSITION,
        VELOCITY,
        CURRENT
    }

    public BreakerSmartMotorController(CANSparkMax sparkMax) {
        this.sparkMax = sparkMax;
        controllerType = ControllerType.SPARK_MAX;
    }

    public BreakerSmartMotorController(BaseMotorController ctreController) {
        this.ctreController = ctreController;
        controllerType = ControllerType.CTRE;
    }

    public void set(SmartMotorControlMode controlMode, double value) {
        set(controlMode, value, false, 0.0);
    }

    public void set(SmartMotorControlMode controlMode, double value, boolean useArbFF, double arbFF) {
        switch (controllerType) {
            case CTRE:
                switch(controlMode) {
                    case POSITION:
                        ctreController.set(ControlMode.Position, value, useArbFF ? DemandType.ArbitraryFeedForward : DemandType.Neutral, arbFF);
                        break;
                    case PRECENT:
                        ctreController.set(ControlMode.PercentOutput, value, useArbFF ? DemandType.ArbitraryFeedForward : DemandType.Neutral, arbFF);
                        break;
                    case VELOCITY:
                        ctreController.set(ControlMode.Velocity, value, useArbFF ? DemandType.ArbitraryFeedForward : DemandType.Neutral, arbFF);
                        break;
                    case VOLTAGE:
                        ctreController.set(ControlMode.PercentOutput, value / RobotController.getBatteryVoltage(), useArbFF ? DemandType.ArbitraryFeedForward : DemandType.Neutral, arbFF);
                        break;
                    case CURRENT:
                        ctreController.set(ControlMode.Current, value, useArbFF ? DemandType.ArbitraryFeedForward : DemandType.Neutral, arbFF);
                        break;
                    default:
                        set(SmartMotorControlMode.PRECENT, value, useArbFF, arbFF);
                        break;
                }
                break;

            case SPARK_MAX:
                arbFF *= RobotController.getBatteryVoltage();
                switch(controlMode) {
                    case POSITION:
                        sparkMax.getPIDController().setReference(value, ControlType.kPosition, pidSlot, useArbFF ? arbFF : 0.0);
                        break;
                    case PRECENT:
                        set(SmartMotorControlMode.VOLTAGE, value * RobotController.getBatteryVoltage(), useArbFF, arbFF);
                        break;
                    case VELOCITY:
                        sparkMax.getPIDController().setReference(value, ControlType.kVelocity, pidSlot, useArbFF ? arbFF : 0.0);
                        break;
                    case VOLTAGE:
                        sparkMax.getPIDController().setReference(value, ControlType.kVoltage, pidSlot, useArbFF ? arbFF : 0.0);
                        break;
                    case CURRENT:
                        sparkMax.getPIDController().setReference(value, ControlType.kCurrent, pidSlot, useArbFF ? arbFF : 0.0);
                        break;
                    default:
                        set(SmartMotorControlMode.PRECENT, value, useArbFF, arbFF);
                        break;
                }
                break;
        }
    }

    public void setPIDSlot(int slot) {
        pidSlot = slot;
        if (controllerType == ControllerType.CTRE) {
            ctreController.selectProfileSlot(slot, 0);
        }
    }

    @Override
    public void set(double speed) {
        this.speed = speed;
        switch(controllerType) {
            case CTRE:
                ctreController.set(ControlMode.PercentOutput, speed);
                break;
            case SPARK_MAX:
                sparkMax.set(speed);
                break;
        } 
    }

    @Override
    public double get() {
        return speed;
    }

    @Override
    public void setInverted(boolean isInverted) {
        this.isInverted = isInverted;
        switch(controllerType) {
            case CTRE:
                ctreController.setInverted(isInverted);
                break;
            case SPARK_MAX:
                sparkMax.setInverted(isInverted);
                break;
        } 
    }

    @Override
    public boolean getInverted() {
        return isInverted;
    }

    @Override
    public void disable() {
        
    }

    @Override
    public void stopMotor() {
        set(0.0);
    }


}
