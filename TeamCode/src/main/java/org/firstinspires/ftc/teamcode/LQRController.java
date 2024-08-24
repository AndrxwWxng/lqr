package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import org.ejml.simple.SimpleMatrix;

@Config
public class LQRController {
    private final SimpleMatrix LQR_Gain_Matrix;

    public SimpleMatrix getAMatrix() {
        return new SimpleMatrix(MatrixConfig.A_MATRIX);
    }

    public SimpleMatrix getBMatrix() {
        return new SimpleMatrix(MatrixConfig.B_MATRIX);
    }

    public SimpleMatrix getQMatrix() {
        return new SimpleMatrix(MatrixConfig.Q_MATRIX);
    }

    public SimpleMatrix getRMatrix() {
        return new SimpleMatrix(MatrixConfig.R_MATRIX);
    }

    @Config
    public static class MatrixConfig {
        // State transition matrix (5x5)
        public static double[][] A_MATRIX = {
                {1, 0, 0, 0.1, 0},
                {0, 1, 0, 0, 0.1},
                {0, 0, 1, 0, 0},
                {0, 0, 0, 1, 0},
                {0, 0, 0, 0, 1}
        };

        // Input matrix (5x4)
        public static double[][] B_MATRIX = {
                {0.005, 0.005, 0.005, 0.005},
                {0.005, -0.005, -0.005, 0.005},
                {0.001, -0.001, 0.001, -0.001},
                {0.1, 0.1, 0.1, 0.1},
                {0.1, -0.1, -0.1, 0.1}
        };

        // State cost matrix (5x5)
        public static double Q_SCALE = 1.0;
        public static double[][] Q_MATRIX = {
                {Q_SCALE, 0, 0, 0, 0},
                {0, Q_SCALE, 0, 0, 0},
                {0, 0, Q_SCALE, 0, 0},
                {0, 0, 0, Q_SCALE, 0},
                {0, 0, 0, 0, Q_SCALE}
        };

        // Input cost matrix (4x4)
        public static double R_SCALE = 0.1;
        public static double[][] R_MATRIX = {
                {R_SCALE, 0, 0, 0},
                {0, R_SCALE, 0, 0},
                {0, 0, R_SCALE, 0},
                {0, 0, 0, R_SCALE}
        };
    }

    public LQRController() {
        SimpleMatrix A = new SimpleMatrix(MatrixConfig.A_MATRIX);
        SimpleMatrix B = new SimpleMatrix(MatrixConfig.B_MATRIX);
        SimpleMatrix Q = new SimpleMatrix(MatrixConfig.Q_MATRIX);
        SimpleMatrix R = new SimpleMatrix(MatrixConfig.R_MATRIX);
        LQR_Gain_Matrix = solveRiccati(A, B, Q, R);
    }

    public SimpleMatrix calculateLQRInput(SimpleMatrix state) {
        return LQR_Gain_Matrix.mult(state).negative();
    }

    private SimpleMatrix solveRiccati(SimpleMatrix A, SimpleMatrix B, SimpleMatrix Q, SimpleMatrix R) {


        SimpleMatrix P = Q;
        int maxIterations = 100;
        double tolerance = 1e-6;

        for (int i = 0; i < maxIterations; i++) {

            SimpleMatrix temp = B.transpose().mult(P).mult(B); // B'PB
            SimpleMatrix temp2 = R.plus(temp); // R + B'PB

            SimpleMatrix temp3 = temp2.invert(); // (R + B'PB)^-1
            SimpleMatrix temp4 = B.mult(temp3).mult(B.transpose()); // B(R + B'PB)^-1B'
            SimpleMatrix P_next = A.transpose().mult(P).mult(A) // A'PA
                    .minus(A.transpose().mult(P).mult(temp4).mult(P).mult(A)) // A'PB(R + B'PB)^-1B'PA
                    .plus(Q); // A'PA - A'PB(R + B'PB)^-1B'PA + Q
            
            if (P_next.minus(P).normF() < tolerance) {
                break;
            }
            P = P_next;
        }


        return R.invert().mult(B.transpose()).mult(P).negative(); // -R^-1B'P
    }
}