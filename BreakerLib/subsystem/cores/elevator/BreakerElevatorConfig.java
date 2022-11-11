// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.elevator;

import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/** Add your docs here. */
public class BreakerElevatorConfig {
    private PIDController pidController;
    private MotorControllerGroup leftGroup;
    private boolean leftInverted;
    private MotorControllerGroup rightGroup;
    private boolean rightInverted;
    private WPI_TalonFX leadMotor;

    public BreakerElevatorConfig(WPI_TalonFX[] leftMotors, boolean invertLeft, WPI_TalonFX[] rightMotors, boolean invertRight, double kP, double kI, 
            double kD, double velocityTolerence, double positionTolerence) {
        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(positionTolerence, velocityTolerence);

        leftGroup = new MotorControllerGroup(leftMotors);
        leftGroup.setInverted(invertLeft);
        leftInverted = invertLeft;

        rightGroup = new MotorControllerGroup(rightGroup);
        rightGroup.setInverted(invertRight);
        rightInverted = invertRight;

        leadMotor = leftMotors[0];
    }

    public PIDController getPidController() {
        return pidController;
    }

    public MotorControllerGroup getLeftMotorGroup() {
        return leftGroup;
    }

    public MotorControllerGroup getRightMotorGroup() {
        return rightGroup;
    }

    public boolean getLeftInverted() {
        return leftInverted;
    }

    public boolean getRightInverted() {
        return rightInverted;
    }

    public WPI_TalonFX getLeadMotor() {
        return leadMotor;
    }

}
