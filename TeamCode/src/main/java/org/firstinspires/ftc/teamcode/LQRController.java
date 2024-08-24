package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import org.ejml.simple.SimpleMatrix;

@Config
public class LQRController {
    private final SimpleMatrix LQR_Gain_Matrix;

    @Config
    public static class MatrixConfig {
        // State transition matrix (6x6)
        public static double[][] A_MATRIX = {
                {1, 0, 0, 0, 0.1, 0},
                {0, 1, 0, 0, 0, 0.1},
                {0, 0, 1, 0, 0, 0},
                {0, 0, 0, 1, 0, 0},
                {0, 0, 0, 0, 1, 0},
                {0, 0, 0, 0, 0, 1}
        };

        // Input matrix (6x4)
        public static double[][] B_MATRIX = {
                {0.005, 0.005, 0.005, 0.005},
                {0.005, -0.005, -0.005, 0.005},
                {0.001, -0.001, 0.001, -0.001},
                {0.02, -0.02, 0.02, -0.02},
                {0.1, 0.1, 0.1, 0.1},
                {0.1, -0.1, -0.1, 0.1}
        };

        // State cost matrix (6x6)
        public static double Q_SCALE = 1.0;
        public static double[][] Q_MATRIX = {
                {Q_SCALE, 0, 0, 0, 0, 0},
                {0, Q_SCALE, 0, 0, 0, 0},
                {0, 0, Q_SCALE, 0, 0, 0},
                {0, 0, 0, Q_SCALE, 0, 0},
                {0, 0, 0, 0, Q_SCALE, 0},
                {0, 0, 0, 0, 0, Q_SCALE}
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

        if (A.getNumCols() != A.getNumRows() || B.getNumRows() != A.getNumRows() || B.getNumCols() != R.getNumRows() ||
                Q.getNumRows() != A.getNumRows() || Q.getNumCols() != A.getNumRows() || R.getNumRows() != R.getNumCols()) {
            throw new IllegalArgumentException("Matrix dimensions are not compatible.");
        }

        SimpleMatrix P = Q;
        int maxIterations = 100;
        double tolerance = 1e-6;

        for (int i = 0; i < maxIterations; i++) {

            SimpleMatrix temp = B.transpose().mult(P).mult(B); // B'PB
            SimpleMatrix temp2 = R.plus(temp); // R + B'PB

            if (temp2.determinant() == 0) {
                throw new RuntimeException("Matrix R + B'PB is singular and cannot be inverted.");
            }

            SimpleMatrix temp3 = temp2.invert(); // (R + B'PB)^-1
            SimpleMatrix temp4 = B.mult(temp3).mult(B.transpose()); // B(R + B'PB)^-1B'
            SimpleMatrix P_next = A.transpose().mult(P).mult(A) // A'PA
                    .minus(A.transpose().mult(P).mult(temp4).mult(P).mult(A)) // A'PB(R + B'PB)^-1B'PA
                    .plus(Q); // A'PA - A'PB(R + B'PB)^-1B'PA + Q

            // Check for convergence
            if (P_next.minus(P).normF() < tolerance) {
                break;
            }
            P = P_next;
        }


        if (R.determinant() == 0) {
            throw new RuntimeException("Matrix R is singular and cannot be inverted.");
        }
        return R.invert().mult(B.transpose()).mult(P).negative(); // -R^-1B'P
    }
}