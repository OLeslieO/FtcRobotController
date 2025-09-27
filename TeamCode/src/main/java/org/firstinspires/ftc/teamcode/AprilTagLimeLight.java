package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;


@TeleOp(name="Limelight_AprilTag", group="Concept")
public class AprilTagLimeLight extends LinearOpMode {

    private static final int TARGET_TAG_ID = 20;   // 目标 AprilTag ID
    private DcMotorEx cameraMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- 初始化底盘 ---
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"leftFrontMotor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"leftBackMotor");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"rightFrontMotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"rightBackMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- 云台电机 ---
        cameraMotor = hardwareMap.get(DcMotorEx.class, "cameraMotor");
        cameraMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cameraMotor.setTargetPosition(0);
        cameraMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cameraMotor.setPower(0.5);

        // Limelight 初始化
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "camera1");
        limelight.setPollRateHz(100);
        limelight.start();

        telemetry.addLine("Press START to begin");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // --- 读取 Limelight 数据 ---
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                for (LLResultTypes.FiducialResult tag : tags) {
                    if (tag.getFiducialId() == TARGET_TAG_ID) {
                        double tx = tag.getTargetXDegrees(); // 水平偏移角度（deg）

                        double Kp = 5.0; // 比例系数
                        int errorTicks = (int)(tx * Kp);
                        int newTarget = cameraMotor.getCurrentPosition() + errorTicks;

                        cameraMotor.setTargetPosition(newTarget);

                        telemetry.addData("Tag ID", tag.getFiducialId());
                        telemetry.addData("tx (deg)", tx);
                        telemetry.addData("Target Pos", newTarget);
                        telemetry.addData("Motor Pos", cameraMotor.getCurrentPosition());
                    }
                }
            } else {
                telemetry.addLine("No AprilTag detected");
            }


            // --- 底盘手动控制 ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double powerCoefficient = 0.5;

            frontLeftMotor.setPower(frontLeftPower * powerCoefficient);
            backLeftMotor.setPower(backLeftPower * powerCoefficient);
            frontRightMotor.setPower(frontRightPower * powerCoefficient);
            backRightMotor.setPower(backRightPower * powerCoefficient);

            telemetry.update();
            sleep(20);
        }
    }
}
