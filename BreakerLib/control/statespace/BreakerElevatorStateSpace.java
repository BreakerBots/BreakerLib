// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.control.statespace;

// import java.security.PublicKey;

// import edu.wpi.first.math.Nat;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.controller.LinearQuadraticRegulator;
// import edu.wpi.first.math.estimator.KalmanFilter;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N2;
// import edu.wpi.first.math.system.LinearSystem;
// import edu.wpi.first.math.system.LinearSystemLoop;
// import edu.wpi.first.math.system.plant.LinearSystemId;

// /** Add your docs here. */
// public class BreakerElevatorStateSpace {
//     private LinearSystem<N2, N1, N1> elevatorPlant;
//     private KalmanFilter<N2, N1, N1> kalmanFilter;
//     private LinearQuadraticRegulator<N2, N1, N1> lqrController;
//     private LinearSystemLoop<N2, N1, N1> loop;

    
//     public BreakerElevatorStateSpace() {
//         elevatorPlant = LinearSystemId.createElevatorSystem(motor, massKg, radiusMeters, G);
//         kalmanFilter = new KalmanFilter<>(Nat.N2(), Nat.N1(), elevatorPlant, VecBuilder.fill(modelKalmanTrust),
//             VecBuilder.fill(encoderKalmanTrust), 0.020);
//         lqrController = new LinearQuadraticRegulator<>(elevatorPlant, qelms, relms, 0.020);
//         loop = new LinearSystemLoop<>(elevatorPlant, lqrController, kalmanFilter, 12.0, 0.020);
//         loop.setNextR(nextR);
//     }


// }
