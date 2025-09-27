package org.firstinspires.ftc.teamcode;
//9月27晚，最新修改的版本，有抖动和缠线问题

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(name = "Limelighttest444)")
public class limelighttest333 extends LinearOpMode {

    private Limelight3A limelight;


    private static final int WINDOW_SIZE = 5;  // 高斯窗口大小
    private final Queue<Double> txHistory = new LinkedList<>();

    int lostcount = 0;

    // 高斯权重
    private final double[] gaussianKernel = {0.06136, 0.24477, 0.38774, 0.24477, 0.06136};

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

            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    double txRaw = tag.getTargetXDegrees();


                    if (txHistory.size() >= WINDOW_SIZE) {
                        txHistory.poll();
                    }
                    txHistory.add(txRaw);

                    double txFiltered = applyGaussianFilter();


                    double deadband = 5.0;

                    if (Math.abs(txFiltered) > deadband) {

                        double kP = 0.02;
                        double power = kP * txFiltered;


                        power = Math.max(-0.1, Math.min(0.1, power));

                        cameraMotor.setPower(power);
                        telemetry.addData("MotorPower", power);
                    } else {
                        cameraMotor.setPower(0);
                        telemetry.addLine("Aligned (within deadband)");
                    }

                    telemetry.addData("TagID", tag.getFiducialId());
                    telemetry.addData("txRaw", txRaw);
                    telemetry.addData("txFiltered", txFiltered);
                    telemetry.addData("ty", tag.getTargetYDegrees());
                }
            } else {
                cameraMotor.setPower(0.1);



            }

            telemetry.update();
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

            double powerCoefficent=1;

            frontLeftMotor.setPower(frontLeftPower*powerCoefficent);
            backLeftMotor.setPower(backLeftPower*powerCoefficent);
            frontRightMotor.setPower(frontRightPower*powerCoefficent);
            backRightMotor.setPower(backRightPower*powerCoefficent);
        }
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
}
