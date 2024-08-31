package org.firstinspires.ftc.teamcode.OpMode;

import org.firstinspires.ftc.teamcode.LQRController;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    private static final double MAX_MOTOR_POWER = 1;
    private static final double ALPHA = 0.1;
    private double[] previousPowers = new double[4];

    @Override
    public void init() {
        initializeHardware(hardwareMap);
        lqrController = new LQRController();

        // Define target positions
        positions.add(new Pose2d(0, 0, Math.toRadians(0)));
        positions.add(new Pose2d(0, 10, Math.toRadians(0)));
        positions.add(new Pose2d(0, 0, Math.toRadians(0)));
    }

    @Override
    public void start() {
        // Reset state for autonomous
        currentPositionIndex = 0;
    }

    @Override
    public void loop() {
        if (currentPositionIndex < positions.size()) {
            Pose2d targetPose = positions.get(currentPositionIndex);
            moveToPosition(targetPose);
            if (hasReachedPosition(targetPose)) {
                currentPositionIndex++;
            }
        } else {
            stopMotors();
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

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
        frontEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightRear")));
    }

    private void moveToPosition(Pose2d targetPose) {
        SimpleMatrix currentState = getCurrentState();
        SimpleMatrix targetState = new SimpleMatrix(new double[][]{
                {targetPose.position.x},
                {targetPose.position.y},
                {targetPose.heading.toDouble()},
                {0} // 0 velocity
        });

        SimpleMatrix lqrInput = lqrController.calculateLQRInput(currentState.minus(targetState));

        double[] limitedPowers = new double[4];
        for (int i = 0; i < 4; i++) {
            double rawPower = lqrInput.get(i);
            double limitedPower = Math.max(-MAX_MOTOR_POWER, Math.min(MAX_MOTOR_POWER, rawPower));
            limitedPowers[i] = ALPHA * limitedPower + (1 - ALPHA) * previousPowers[i];
            previousPowers[i] = limitedPowers[i];
        }

        setMotorPowers(limitedPowers);
    }

    private SimpleMatrix getCurrentState() {
        double leftPosition = leftEncoder.getPositionAndVelocity().position;
        double frontPosition = frontEncoder.getPositionAndVelocity().position;
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double velocity = (leftEncoder.getPositionAndVelocity().velocity +
                frontEncoder.getPositionAndVelocity().velocity) / 2;

        return new SimpleMatrix(new double[][]{
                {leftPosition},
                {frontPosition},
                {heading},
                {velocity}
        });
    }

    private boolean hasReachedPosition(Pose2d targetPose) {
        SimpleMatrix currentState = getCurrentState();
        double tolerance = 1.0; // Define an appropriate tolerance
        return Math.abs(currentState.get(0) - targetPose.position.x) < tolerance &&
                Math.abs(currentState.get(1) - targetPose.position.y) < tolerance &&
                Math.abs(currentState.get(2) - targetPose.heading.toDouble()) < Math.toRadians(5);
    }

    private void stopMotors() {
        setMotorPowers(new double[]{0, 0, 0, 0});
    }

    private void setMotorPowers(double[] powers) {
        frontLeftMotor.setPower(powers[0]);
        backLeftMotor.setPower(powers[1]);
        frontRightMotor.setPower(powers[2]);
        backRightMotor.setPower(powers[3]);
    }
}
