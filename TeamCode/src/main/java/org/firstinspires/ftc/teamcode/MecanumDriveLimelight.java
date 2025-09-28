package org.firstinspires.ftc.teamcode;
//在cyz发的limelight111版本的基础上加了lasttx，用来判断丢失时云台转的方向
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(name = "Limelighttest1113")
public class MecanumDriveLimelight extends LinearOpMode {

    private Limelight3A limelight;

    private static final int WINDOW_SIZE = 6;  // 高斯窗口增大一点
    private final Queue<Double> txHistory = new LinkedList<>();
    private Double lastTx = null; // 上一次检测到的tx

    int lostcount = 0;

    // 高斯权重
    private final double[] gaussianKernel = {0.06136, 0.24477, 0.38774, 0.24477, 0.06136};
    private double previousTxFiltered = 0;  // 搞一个低通滤波器

    // 新增变量 - 调整后的参数
    private double previousRx = 0; // 上一次的旋转值
    private double cameraRotationCompensation = 0; // 摄像头旋转补偿量
    private static final double COMPENSATION_RATIO = 0.6; // 降低补偿比例系数
    private static final double MIN_ROTATION_THRESHOLD = 0.1; // 忽略小的旋转输入
    private static final double MAX_COMPENSATION_POWER = 0.15; // 降低最大补偿功率

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx cameraMotor = hardwareMap.get(DcMotorEx.class, "cameraMotor");
        cameraMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "camera1");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(7);
        limelight.start();

        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"leftFrontMotor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"leftBackMotor");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"rightFrontMotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"rightBackMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.[]\
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        int notfound = 0;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            // 获取机器人旋转速度（右摇杆X轴）
            double rx = gamepad1.right_stick_x;

            // 计算摄像头需要补偿的旋转量
            double rotationCompensation = calculateRotationCompensation(rx);

            // 处理视觉跟踪
            double visualPower = 0;
            boolean targetFound = false;

            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    double txRaw = tag.getTargetXDegrees();
                    lastTx = txRaw; // 更新最后一次检测到的tx

                    if (txHistory.size() >= WINDOW_SIZE) txHistory.poll();
                    txHistory.add(txRaw);

                    double txFiltered = applyGaussianFilter();
                    txFiltered = applyLowPassFilter(txFiltered);

                    double deadband = 7.0;

                    if (Math.abs(txFiltered) > deadband) {
                        double kP = 0.008; // 进一步减少视觉跟踪幅度
                        visualPower = kP * txFiltered;
                        visualPower = Math.max(-0.15, Math.min(0.15, visualPower)); // 降低视觉跟踪功率限制
                    } else {
                        visualPower = 0;
                        telemetry.addLine("Aligned (within deadband)");
                    }

                    targetFound = true;
                    telemetry.addData("TagID", tag.getFiducialId());
                    telemetry.addData("txRaw", txRaw);
                    telemetry.addData("txFiltered", txFiltered);
                    telemetry.addData("ty", tag.getTargetYDegrees());
                }
            } else {
                // 根据 lastTx 决定方向
                if (lastTx != null) {
                    double lostPower = 0.1; // 进一步减小搜索功率
                    if (lastTx > 0) {
                        //最后在右边往左转
                        visualPower = -lostPower;
                    } else {
                        //最后在左边往右转
                        visualPower = lostPower;
                    }
                } else {
                    visualPower = 0.15; // 降低默认搜索功率
                }
            }

            // 合并视觉跟踪功率和旋转补偿功率
            double totalPower = visualPower + rotationCompensation;

            // 限制总功率在安全范围内
            totalPower = Math.max(-0.3, Math.min(0.3, totalPower));

            // 设置摄像头电机功率
            cameraMotor.setPower(totalPower);

            telemetry.addData("Visual Power", visualPower);
            telemetry.addData("Rotation Compensation", rotationCompensation);
            telemetry.addData("Total Power", totalPower);
            telemetry.addData("Robot Rotation (rx)", rx);

            telemetry.update();

            // 驱动代码保持不变
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double powerCoefficent= 0.8;

            frontLeftMotor.setPower(frontLeftPower*powerCoefficent);
            backLeftMotor.setPower(backLeftPower*powerCoefficent);
            frontRightMotor.setPower(frontRightPower*powerCoefficent);
            backRightMotor.setPower(backRightPower*powerCoefficent);
        }
    }

    /**
     * 计算摄像头旋转补偿量
     * 当机器人旋转时，摄像头需要反向旋转以保持对目标的跟踪
     */
    private double calculateRotationCompensation(double currentRx) {
        // 忽略小的旋转输入（死区）
        if (Math.abs(currentRx) < MIN_ROTATION_THRESHOLD) {
            previousRx = currentRx;
            return 0;
        }

        // 计算旋转变化率
        double rotationDelta = currentRx - previousRx;

        // 应用补偿（负号表示反向补偿），使用更低的补偿比例
        double compensation = -currentRx * COMPENSATION_RATIO;

        // 限制补偿量在更低的范围内
        compensation = Math.max(-MAX_COMPENSATION_POWER, Math.min(MAX_COMPENSATION_POWER, compensation));

        // 更新上一次的旋转值
        previousRx = currentRx;

        return compensation;
    }

    private double applyGaussianFilter() {
        if (txHistory.isEmpty()) return 0.0;

        Double[] values = txHistory.toArray(new Double[0]);
        int size = values.length;
        double sum = 0;

        for (int i = 0; i < size; i++) {
            // 权重对齐到最新值（右端）
            int kernelIndex = gaussianKernel.length - size + i;
            if (kernelIndex < 0) kernelIndex = 0;
            sum += values[i] * gaussianKernel[kernelIndex];
        }

        return sum;
    }

    private double applyLowPassFilter(double currentTxFiltered) {
        double alpha = 0.1;  // 平滑系数，值越小越平滑
        double filtered = alpha * currentTxFiltered + (1 - alpha) * previousTxFiltered;
        previousTxFiltered = filtered;
        return filtered;
    }
}