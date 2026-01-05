package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.Timer;

public class WingPoseEstimator {
    private KalmanFilter<N3, N3, N3> wingPoseKF;
    private LinearSystem<N3, N3, N3> plant;
    private Matrix<N3, N1> stateStdDevs;
    private Matrix<N3, N1> measurementStdDevs;
    private final Matrix<N3, N1> ZERO_MATRIX = new Matrix<>(Nat.N3(), Nat.N1());
     
    public WingPoseEstimator() {
        Nat<N3> matrixSize = Nat.N3();
        Matrix<N3, N3> A = Matrix.eye(matrixSize); // state does not propogate as object is stationary (zero velocity)
        Matrix<N3, N3> B = new Matrix<>(matrixSize, matrixSize); // no inputs (e.g. odometry/control)
        Matrix<N3, N3> C = Matrix.eye(matrixSize); // all states are measured directly
        Matrix<N3, N3> D = new Matrix<>(matrixSize, matrixSize); // again, no inputs. also no feedthrough anyway

        plant = new LinearSystem<>(A, B, C, D);

        //dummy values
        stateStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);
        measurementStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);

        //construct a linear system plant with my guess of what the states, inputs, and outputs are
        plant = new LinearSystem<N3, N3, N3>(A, B, C, D);

        // intialize kalman filter with 3 states, 3 inputs, and 3 outputs
        // technically no inputs, but we have to put something in
        wingPoseKF = new KalmanFilter<N3, N3, N3>(matrixSize, matrixSize, plant, stateStdDevs, measurementStdDevs, 0.02);   
    }

    public void addVisionMeasurement(Pose2d input) {
        double x = input.getX();
        double y = input.getY();
        double theta = input.getRotation().getRadians(); //TODO: account for wraparound (if necessary)

        Matrix<N3, N1> measurement = VecBuilder.fill(x, y, theta);
        wingPoseKF.correct(ZERO_MATRIX, measurement); // no control input
    }

    public void addVisionMeasurement(Pose2d input, Matrix<N3, N1> measurementStdDevs) {
        double x = input.getX();
        double y = input.getY();
        double theta = input.getRotation().getRadians(); //TODO: account for wraparound (if necessary)

        Matrix<N3, N1> measurement = VecBuilder.fill(x, y, theta);

        var covarianceMatrix = StateSpaceUtil.makeCovarianceMatrix(Nat.N3(), measurementStdDevs); // create covariance matrix from std devs
        
        wingPoseKF.correct(ZERO_MATRIX, measurement, covarianceMatrix); // no control input
    }

    public void periodic(double loopTime) {
        wingPoseKF.predict(ZERO_MATRIX, loopTime); // no control input
    }

    public Pose2d getEstimatedPose() {
        double x = wingPoseKF.getXhat(0);
        double y = wingPoseKF.getXhat(1);
        double theta = wingPoseKF.getXhat(2);

        Pose2d estimatedPose = new Pose2d(x, y, new Rotation2d(theta));
        return estimatedPose;
    }
}