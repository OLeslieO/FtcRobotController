package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="MecanumWithTagTracking")
public class AprilTagLimeLight extends LinearOpMode {

    private GoBildaPinpointDriver odo;
    private DcMotorEx cameraMotor;
    private Limelight3A limelight;

    private static final int TARGET_TAG_ID = 20; // 目标 AprilTag
    private static final double TICKS_PER_DEGREE = 10; // 根据云台电机编码器测试

    @Override
    public void runOpMode() throws InterruptedException {

        // 初始化硬件
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        cameraMotor = hardwareMap.get(DcMotorEx.class, "cameraMotor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight3A");

        // 初始化 odo
        odo.setOffsets(100,90, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Pose2D pose2D = new Pose2D(DistanceUnit.MM,-1420,-1350, AngleUnit.DEGREES, Math.toRadians(0));
        odo.setPosition(pose2D);

        // 初始化云台电机
        cameraMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        cameraMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        cameraMotor.setTargetPosition(0);
        cameraMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        cameraMotor.setPower(1);

        // 初始化底盘电机
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"leftFrontMotor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"leftBackMotor");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"rightFrontMotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"rightBackMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {

            // 更新里程计
            odo.update();

            double localx = odo.getPosition().getX(DistanceUnit.MM);
            double localy = odo.getPosition().getY(DistanceUnit.MM);
            double headingAngle = odo.getPosition().getHeading(DEGREES);

            telemetry.addData("x", localx);
            telemetry.addData("y", localy);
            telemetry.addData("heading", headingAngle);

            // -----------------------
            // 云台自动跟随 AprilTag
            // -----------------------
            // Limelight 输出示例：
            // - getTID(): 检测到的 Tag ID
            // - getTX(): 水平偏差，单位为度（目标在画面中偏离中心的角度）
            int detectedID = limelight.getTID();
            if(detectedID == TARGET_TAG_ID) {
                double tx; // 水平偏差
                tx = limelight.getTX();
                double kP = 0.5; // 比例控制系数
                int targetPos = (int)(tx * kP * TICKS_PER_DEGREE); // 转换为编码器目标位置
                cameraMotor.setTargetPosition(targetPos);

                telemetry.addData("TargetID", detectedID);
                telemetry.addData("tx(deg)", tx);
                telemetry.addData("Camera Motor Pos", cameraMotor.getCurrentPosition());
                telemetry.addData("Camera Target Pos", cameraMotor.getTargetPosition());
            }

            // -----------------------
            // 底盘 mecanum 控制
            // -----------------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.update();
            sleep(20);
        }
    }
}
