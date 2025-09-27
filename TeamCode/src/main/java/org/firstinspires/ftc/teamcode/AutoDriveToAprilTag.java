package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp(name="Auto Drive Limelight + Pan Motor", group="Concept")
public class AutoDriveToAprilTag extends LinearOpMode {

    private static final int TARGET_TAG_ID = 23;
    private static final double DESIRED_DISTANCE = 12.0; // inch
    private static final double DISTANCE_GAIN = 0.02;
    private static final double TURN_GAIN = 0.01;
    private static final double MAX_SPEED = 0.5;
    private static final double MAX_TURN = 0.3;
    private static final double DISTANCE_TOLERANCE = 0.5; // inch
    private static final double ANGLE_TOLERANCE = 1.0; // degree
    private static final double PAN_GAIN = 0.5; // tx -> panMotor功率比例
    private static final double MAX_PAN_POWER = 0.5; // 最大云台功率

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx panMotor;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        // 初始化底盘电机
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBackMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBackMotor");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // 初始化云台电机
        panMotor = hardwareMap.get(DcMotorEx.class, "cameraMotor");
        panMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        panMotor.setDirection(DcMotor.Direction.FORWARD);

        // 初始化 Limelight
        limelight = hardwareMap.get(Limelight3A.class, "camera1");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(7); // 设置 pipeline，确保支持 AprilTag

        telemetry.addLine("Press START to begin");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            double drive = 0, turn = 0;
            double panPower = 0;
            boolean targetVisible = false;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    for (LLResultTypes.FiducialResult tag : tags) {
                        if (tag.getFiducialId() == TARGET_TAG_ID) {
                            targetVisible = true;
                            double tx = tag.getTargetXDegrees(); // 水平偏角
                            double ta = tag.getTargetArea();      // 面积近似距离

                            // 旋转控制（底盘）
                            turn = Range.clip(tx * TURN_GAIN, -MAX_TURN, MAX_TURN);

                            // 前进/后退控制（面积近似距离）
                            double distanceError = DESIRED_DISTANCE - ta;
                            drive = Range.clip(distanceError * DISTANCE_GAIN, -MAX_SPEED, MAX_SPEED);

                            // 云台电机控制

                            panMotor.setPower(0.3);

                            telemetry.addData("Tag ID", tag.getFiducialId());
                            telemetry.addData("tx", tx);
                            telemetry.addData("ta", ta);
                            telemetry.addData("Drive", drive);
                            telemetry.addData("Turn", turn);
                            telemetry.addData("PanPower", panPower);

                            // 停止条件
                            if (Math.abs(distanceError) < DISTANCE_TOLERANCE && Math.abs(tx) < ANGLE_TOLERANCE) {
                                drive = 0;
                                turn = 0;
                                panMotor.setPower(0);
                            }

                            break; // 找到目标就退出循环
                        }
                    }
                }
            }

            if (!targetVisible) {
                telemetry.addData("Limelight", "No target");
                drive = 0;
                turn = 0;
                panMotor.setPower(0);
            }

            telemetry.update();
            moveRobot(drive, turn);
            sleep(20);
        }
    }

    private void moveRobot(double forward, double rotate) {
        // Mecanum 驱动公式，取消横移
        double frontLeftPower = forward - rotate;
        double frontRightPower = forward + rotate;
        double backLeftPower = forward - rotate;
        double backRightPower = forward + rotate;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }
}
