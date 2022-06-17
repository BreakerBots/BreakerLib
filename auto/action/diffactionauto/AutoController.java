package frc.robot.BreakerLib.auto.action.diffactionauto;

import edu.wpi.first.math.controller.PIDController;

/** Non-trajectory auto controller. If you're into wasting your potential. */
public class AutoController {
    private static double pivotFF = 0;
    private static double pivotKp = 0;
    private static double pivotKi = 0;
    private static double pivotKd = 0;
    private static double pivotPosTol = 0;
    private static double pivotVelTol = 0;

    private static double moveStraightFF = 0;
    private static double moveStraightKp = 0;
    private static double moveStraightKi = 0;
    private static double moveStraightKd = 0;
    private static double moveStraightPosTol = 0;
    private static double moveStraightVelTol = 0;
    public PIDController pivotPID;
    public PIDController moveStraightPID;

    public static final void setPivotPIDVals(double feedForward, double Kp, double Ki, double Kd, double posTol, double velTol) {
        pivotFF = feedForward;
        pivotKp = Kp;
        pivotKi = Ki;
        pivotKd = Kd;
        pivotPosTol = posTol;
        pivotVelTol = velTol;
    }

    public static final void setMoveForwardPIDVals(double feedForward, double Kp, double Ki, double Kd, double posTol, double velTol) {
        moveStraightFF = feedForward;
        moveStraightKp = Kp;
        moveStraightKi = Ki;
        moveStraightKd = Kd;
        moveStraightPosTol = posTol;
        moveStraightVelTol = velTol;
    }

    public AutoController() {
        pivotPID = new PIDController(pivotKp, pivotKi, pivotKd);
        pivotPID.setTolerance(pivotPosTol, pivotVelTol);
        moveStraightPID = new PIDController(moveStraightKp, moveStraightKi, moveStraightKd);
        moveStraightPID.setTolerance(moveStraightPosTol, moveStraightVelTol);
    }

    public double getPivotFeedForward() {
        return pivotFF;
    }

    public double getMoveStraightFeedForward() {
        return moveStraightFF;
    }

    public double calculatePivotPID(double curAng, double setAng) {
       return pivotPID.calculate(curAng, setAng);
    }

    public double calculateMoveStraightPID(double curDist, double setDist) {
        return moveStraightPID.calculate(curDist, setDist);
    }

    public double getPivotError() {
        return pivotPID.getPositionError();
    }

    public double getMoveStraightError() {
        return moveStraightPID.getPositionError();
    }
    
    public boolean atPivotSetPoint() {
        return pivotPID.atSetpoint();
    }
    
    public boolean atMoveStraightSetPoint() {
        return moveStraightPID.atSetpoint();
    }


    
    


}
