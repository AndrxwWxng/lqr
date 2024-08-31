package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.ejml.simple.SimpleMatrix;

@Config
public
class LQRController {
    private final SimpleMatrix LQR_Gain_Matrix;


    @Config
    public static class MatrixConfig {
        public static double[][] A_MATRIX = {
                {1, 0, 0, 0, 0.1, 0},
                {0, 1, 0, 0, 0, 0.1},
                {0, 0, 1, 0, 0, 0},
                {0, 0, 0, 1, 0, 0},
                {0, 0, 0, 0, 1, 0},
                {0, 0, 0, 0, 0, 1}
        };

        public static double[][] B_MATRIX = {
                {0.005, 0.005, 0.005, 0.005, 0, 0},
                {0.005, -0.005, -0.005, 0.005, 0, 0},
                {0.001, -0.001, 0.001, -0.001, 0, 0},
                {0.02, -0.02, 0.02, -0.02, 0, 0},
                {0.1, 0.1, 0.1, 0.1, 0, 0},
                {0.1, -0.1, -0.1, 0.1, 0, 0}
        };

        public static double Q_SCALE = 1.0;
        public static double R_SCALE = 0.1;
    }

    public LQRController() {
//        TODO NEED TO TUNE THESE
//         State transition matrix
        SimpleMatrix A = new SimpleMatrix(MatrixConfig.A_MATRIX);

        // Input matrix
        SimpleMatrix B = new SimpleMatrix(MatrixConfig.B_MATRIX);

        // State cost matrix
        SimpleMatrix Q = SimpleMatrix.identity(6).scale(MatrixConfig.Q_SCALE);

        // Input cost matrix
        SimpleMatrix R = SimpleMatrix.identity(4).scale(MatrixConfig.R_SCALE);

        LQR_Gain_Matrix = solveRiccati(A, B, Q, R);
    }

    public SimpleMatrix calculateLQRInput(SimpleMatrix state) {
        return LQR_Gain_Matrix.mult(state).negative();
    }

    private SimpleMatrix solveRiccati(SimpleMatrix A, SimpleMatrix B, SimpleMatrix Q, SimpleMatrix R) {
        SimpleMatrix P = Q;
        for (int i = 0; i < 100; i++) {
            SimpleMatrix temp = P.mult(B).mult(R.invert()).mult(B.transpose()).mult(P);
            P = A.transpose().mult(P).mult(A).minus(temp).plus(Q);
        }
        return R.invert().mult(B.transpose()).mult(P);
    }
}