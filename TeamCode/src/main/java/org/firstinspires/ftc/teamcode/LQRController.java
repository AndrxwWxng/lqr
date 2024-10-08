package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import org.ejml.simple.SimpleMatrix;

@Config
public class LQRController {
    private SimpleMatrix LQR_Gain_Matrix;

    @Config
    public static class MatrixConfig {
        // State transition matrix (4x4)
        public static double[][] A_MATRIX = {
                {1, 0, 0, 0},
                {0, 1, 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
        };

        // Input matrix (4x4)
        public static double[][] B_MATRIX = {
                {0.005, 0.005, 0.005, 0.005},
                {0.005, -0.005, -0.005, 0.005},
                {0.001, -0.001, 0.001, -0.001},
                {0.001, 0.001, -0.001, -0.001}
        };

        // Cost matrix for state (Q) (4x4)
        public static double[][] Q_MATRIX = {
                {1, 0, 0, 0},
                {0, 1, 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
        };

        // Cost matrix for control input (R) (4x4)
        public static double[][] R_MATRIX = {
                {1, 0, 0, 0},
                {0, 1, 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
        };

        public static double CONVERGENCE_THRESHOLD = 1e-4;
        public static int MAX_ITERATIONS = 100;
    }

    public LQRController() {
        SimpleMatrix A = new SimpleMatrix(MatrixConfig.A_MATRIX);
        SimpleMatrix B = new SimpleMatrix(MatrixConfig.B_MATRIX);
        SimpleMatrix Q = new SimpleMatrix(MatrixConfig.Q_MATRIX);
        SimpleMatrix R = new SimpleMatrix(MatrixConfig.R_MATRIX);

        LQR_Gain_Matrix = solveRiccati(A, B, Q, R);
    }

    public SimpleMatrix calculateLQRInput(SimpleMatrix stateError) {
        return LQR_Gain_Matrix.mult(stateError).negative();
    }

    private SimpleMatrix solveRiccati(SimpleMatrix A, SimpleMatrix B, SimpleMatrix Q, SimpleMatrix R) {
        SimpleMatrix P = Q;
        SimpleMatrix BT = B.transpose();
        SimpleMatrix AT = A.transpose();
        SimpleMatrix Rinv = R.invert();

        for (int i = 0; i < MatrixConfig.MAX_ITERATIONS; i++) {
            SimpleMatrix K = Rinv.mult(BT).mult(P);
            SimpleMatrix newP = AT.mult(P).mult(A).minus(AT.mult(P).mult(B).mult(K)).plus(Q);

            if (P.minus(newP).elementMaxAbs() < MatrixConfig.CONVERGENCE_THRESHOLD) {
                break;
            }
            P = newP;
        }

        return Rinv.mult(BT).mult(P);
    }

    public SimpleMatrix getLQRGainMatrix() {
        return LQR_Gain_Matrix;
    }
}
