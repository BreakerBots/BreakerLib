// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics.projectilemotion;

import javax.crypto.spec.GCMParameterSpec;

import com.fasterxml.jackson.databind.deser.impl.CreatorCandidate;

import frc.robot.BreakerLib.util.math.BreakerUnits;

/** A class that represents the properties of a projectile such as a ball */
public class BreakerProjectile {
    private double massKg;
    private double dragCoeffiecnt;
    private double crossSectionalAreaMetersSq;
    private double termanalVelMetersPerSec;
    private double drag;
    private double termanalVelSq;
    public BreakerProjectile(double massKg, double dragCoeffiecnt, double crossSectionalAreaMetersSq, double termanalVelMetersPerSec) {
        this.massKg = massKg;
        this.dragCoeffiecnt = dragCoeffiecnt;
        this.crossSectionalAreaMetersSq = crossSectionalAreaMetersSq;
        termanalVelMetersPerSec = Math.sqrt((2 * massKg * BreakerUnits.METERS_PER_SECOND_SQUARED_IN_G) / (dragCoeffiecnt * BreakerUnits.AIR_DENCITY_KG_PER_METER_CUBED * crossSectionalAreaMetersSq));
        termanalVelSq = Math.pow(termanalVelMetersPerSec, 2);
        drag = 0.5 * dragCoeffiecnt * BreakerUnits.AIR_DENCITY_KG_PER_METER_CUBED * crossSectionalAreaMetersSq * termanalVelSq;
    }
    
    public double getCrossSectionalAreaMetersSq() {
        return crossSectionalAreaMetersSq;
    }

    public double getDrag() {
        return drag;
    }

    public double getDragCoeffiecnt() {
        return dragCoeffiecnt;
    }

    public double getMassKg() {
        return massKg;
    }

    public double getTermanalVelMetersPerSec() {
        return termanalVelMetersPerSec;
    }

    public double getTerminalVelSq() {
        return termanalVelSq;
    }
}
