package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;


import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name="AprilTag", group="Concept")

public class AprilTag extends LinearOpMode {


    private static final int TARGET_TAG_ID = 20;   // 目标 AprilTag ID BLUE20/RED24


    private DcMotorEx cameraMotor;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private double angle;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"leftFrontMotor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"leftBackMotor");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"rightFrontMotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"rightBackMotor");
        cameraMotor = hardwareMap.get(DcMotorEx.class, "cameraMotor");
        cameraMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        cameraMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cameraMotor.setTargetPosition(0);
        cameraMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cameraMotor.setPower(1);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //double CPR = 100 ;
        int position = cameraMotor.getCurrentPosition();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "camera1");
        limelight.setPollRateHz(100); // 这设置了我们向Limelight请求数据的频率（每秒100次）
        limelight.start(); // 这告诉Limelight开始观察！




        // 初始化 AprilTag 处理器
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Press START to begin");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {



            List<AprilTagDetection> detections = aprilTag.getDetections();
            AprilTagDetection targetTag = null;

            // 查找目标标签
            for (AprilTagDetection d : detections) {
                if (d.id == TARGET_TAG_ID) {
                    targetTag = d;
                    break;
                }
            }
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double powerCoefficent=0.5;

            frontLeftMotor.setPower(frontLeftPower*powerCoefficent);
            backLeftMotor.setPower(backLeftPower * powerCoefficent);
            frontRightMotor.setPower(frontRightPower * powerCoefficent);
            backRightMotor.setPower(backRightPower*powerCoefficent);

            if (targetTag != null && targetTag.robotPose != null) {
                // 获取目标标签的 yaw，相对摄像头方向
                double yaw = targetTag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                // 计算电机目标位置（保留你原来的风格）
                // 这里可以加一个偏移量，比如摄像头初始角度
                double angle;  // 可以加偏移: angle = yaw + offset;
                angle = yaw;
                int targetPos = (int) angle;

                cameraMotor.setTargetPosition(targetPos);

                telemetry.addData("Target Tag ID", targetTag.id);
                telemetry.addData("Yaw (deg)", yaw);
                telemetry.addData("Motor Target Pos", targetPos);
                telemetry.addData("Motor Current Pos", cameraMotor.getCurrentPosition());
            } else {
                telemetry.addLine("Target Tag NOT visible");
            }


            telemetry.update();

            sleep(10);
        }

        // 结束后关闭摄像头
        visionPortal.close();
    }
}
