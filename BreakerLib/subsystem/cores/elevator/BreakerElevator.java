// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerUnits;

/** Add your docs here. */
public class BreakerElevator extends SubsystemBase {
    private PIDController pid;

    private MotorControllerGroup leftGroup;
    private MotorControllerGroup rightGroup;
    private WPI_TalonFX leadMotor;

    private double elevatorGearRatio;
    private double elevatorDrumRadius;
    private DoubleSupplier elevatorHightSupplierMeters;
    private boolean usesSupplier;

    private double currentHightMeters = 0;
    private double setHightMeters = 0;

    private boolean limitsEnabled = false;

    private BooleanSupplier fLimit;
    private DigitalInput fLimitDIO;
    private boolean invertLimitF = false;
    private boolean hasLimitF = false;

    private BooleanSupplier rLimit;
    private DigitalInput rLimitDIO;
    private boolean invertLimitR = false;
    private boolean hasLimitR = false;

    /** Uses integrated motor encoder */
    public BreakerElevator(double elevatorGearRatio, double elevatorDrumRadius, BreakerElevatorConfig config) {
        this.elevatorGearRatio = elevatorGearRatio;
        this.elevatorDrumRadius = elevatorGearRatio;
        usesSupplier = false;

        pid = config.getPidController();

        leftGroup = config.getLeftMotorGroup();
        rightGroup = config.getRightMotorGroup();
        leadMotor = config.getLeadMotor();
    }

    /** Uses integrated motor encoder */
    public BreakerElevator(double elevatorGearRatio, DoubleSupplier elevatorHightSupplierMeters, BreakerElevatorConfig config) {
        this.elevatorHightSupplierMeters = elevatorHightSupplierMeters;
        usesSupplier = true;
    
        pid = config.getPidController();
    
        leftGroup = config.getLeftMotorGroup();
        rightGroup = config.getRightMotorGroup();
        leadMotor = config.getLeadMotor();
    }


    public void setHight(double setHightMeters) {
        this.setHightMeters = setHightMeters;
    }

    private void calculateCurrentHight() {
        if (usesSupplier) {
            currentHightMeters = elevatorHightSupplierMeters.getAsDouble();
        } else {
            currentHightMeters = Units.inchesToMeters(BreakerMath.ticksToInches(leadMotor.getSelectedSensorPosition(), BreakerMath.getTicksPerInch(2048.0, elevatorGearRatio, elevatorDrumRadius)));
        }
    }

    public void addForwardLimitSwitch(int dioChannel) {
        fLimitDIO = new DigitalInput(dioChannel);
        fLimit = fLimitDIO :: get;
        invertLimitF = false;
        hasLimitF = true;
    }

    public void addForwardLimitSwitch(int dioChannel, boolean invertInput) {
        fLimitDIO = new DigitalInput(dioChannel);
        fLimit = fLimitDIO :: get;
        invertLimitF = invertInput;
        hasLimitF = true;
    }

    public void addReverceLimitSwitch(int dioChannel) {
        rLimitDIO = new DigitalInput(dioChannel);
        rLimit = rLimitDIO :: get;
        invertLimitR = false;
        hasLimitR = true;
    }

    public void addReverceLimitSwitch(int dioChannel, boolean invertInput) {
        rLimitDIO = new DigitalInput(dioChannel);
        rLimit = rLimitDIO :: get;
        invertLimitR = invertInput;
        hasLimitR = true;
    }

    public void addForwardLimitSwitch(BooleanSupplier switchSupplier) {
        fLimit = switchSupplier;
        invertLimitF = false;
        hasLimitF = true;
    }

    public void addForwardLimitSwitch(BooleanSupplier switchSupplier, boolean invertInput) {
        fLimit = switchSupplier;
        invertLimitF = false;
        hasLimitF = true;
    }

    public void addReverceLimitSwitch(BooleanSupplier switchSupplier) {
        rLimit = switchSupplier;
        invertLimitR = false;
        hasLimitR = true;
    }

    public void addReverceLimitSwitch(BooleanSupplier switchSupplier, boolean invertInput) {
        rLimit = switchSupplier;
        invertLimitR = false;
        hasLimitR = true;
    }

    public void enableLimits(boolean isEnabled) {
        limitsEnabled = isEnabled;
    }

    public void setManualPrecentSpeed(double precentSpeed) {
        leftGroup.set(precentSpeed);
        rightGroup.set(precentSpeed);
    }

    public double getElevatorHightMeters() {
        return currentHightMeters;
    }

    public double getCurrentSetHightMeters() {
        return setHightMeters;
    }

    private boolean checkLimits() {
        if (limitsEnabled) {
            if (hasLimitF) {
                return invertLimitF ? !fLimit.getAsBoolean() : fLimit.getAsBoolean();
            } else if (hasLimitR) {
                return invertLimitR ? !rLimit.getAsBoolean() : rLimit.getAsBoolean();
            }
        }
        return false;
    }

    private void logicLoop() {
        calculateCurrentHight();
        double calculatedSpeed = 0;
        if (!checkLimits()) {
            calculatedSpeed = pid.calculate(currentHightMeters, setHightMeters);
        }
        setManualPrecentSpeed(calculatedSpeed);
    }

    @Override
    public void periodic() {
       logicLoop();
    }

}
