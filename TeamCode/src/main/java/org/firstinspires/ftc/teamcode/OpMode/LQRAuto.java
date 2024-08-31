package org.firstinspires.ftc.teamcode.OpMode;

import org.firstinspires.ftc.teamcode.LQRController;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class LQRAuto extends OpMode {

    private DcMotorEx backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor;
    private Encoder leftEncoder, frontEncoder;
    private IMU imu;
    private LQRController lqrController;

    private List<Pose2d> positions = new ArrayList<>();
    private int currentPositionIndex = 0;

    @Override
    public void init() {

        initializeHardware(hardwareMap);
        lqrController = new LQRController();


        positions.add(new Pose2d(0, 0, Math.toRadians(0)));
        positions.add(new Pose2d(50, 0, Math.toRadians(0)));
        positions.add(new Pose2d(0, 0, Math.toRadians(0)));
    }

    @Override
    public void loop() {
        if (currentPositionIndex < positions.size()) {
            Pose2d targetPose = positions.get(currentPositionIndex);
            moveToPosition(targetPose.position.x, targetPose.position.y, targetPose.heading.toDouble());

            if (isAtTargetPosition(targetPose.position.x, targetPose.position.y, targetPose.heading.toDouble())) {
                //only moves on if the position is accurate
                currentPositionIndex++;
            }
        } else {
            //stops when all positions are done
            setMotorPowers(new double[]{0, 0, 0, 0});
        }
    }

    private void initializeHardware(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");

        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftRear")));
        frontEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightFront")));
    }

    private void moveToPosition(double targetX, double targetY, double targetHeading) {
        SimpleMatrix currentState = getCurrentState();
        SimpleMatrix targetState = new SimpleMatrix(new double[][]{
                {targetX},
                {targetY},
                {targetHeading},
                {0}, // target velocity = 0
                {0}  // for matching matrix dimensions
        });

        SimpleMatrix lqrInput = lqrController.calculateLQRInput(currentState.minus(targetState));

        setMotorPowers(new double[]{
                lqrInput.get(0),
                lqrInput.get(1),
                lqrInput.get(2),
                lqrInput.get(3)
        });

        telemetry.addData("Target Pose", targetX + ", " + targetY + ", " + Math.toDegrees(targetHeading));
        telemetry.addData("Current X", currentState.get(0));
        telemetry.addData("Current Y", currentState.get(1));
        telemetry.addData("LQR Input", lqrInput.toString());
        telemetry.update();
    }

    private boolean isAtTargetPosition(double targetX, double targetY, double targetHeading) {
        SimpleMatrix currentState = getCurrentState();
        double tolerance = 0.5; //can adjust this
        return Math.abs(currentState.get(0) - targetX) < tolerance &&
                Math.abs(currentState.get(1) - targetY) < tolerance &&
                Math.abs(currentState.get(2) - targetHeading) < Math.toRadians(2); // 2 degrees tolerance
    }

    private SimpleMatrix getCurrentState() {
        double leftPosition = leftEncoder.getPositionAndVelocity().position;
        double frontPosition = frontEncoder.getPositionAndVelocity().position;
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        return new SimpleMatrix(new double[][]{
                {leftPosition},
                {frontPosition},
                {heading},
                {0},
                {0}
        });
    }

    private void setMotorPowers(double[] powers) {
        frontLeftMotor.setPower(powers[0]);
        backLeftMotor.setPower(powers[1]);
        frontRightMotor.setPower(powers[2]);
        backRightMotor.setPower(powers[3]);
    }
}
