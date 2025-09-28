//lucas在27晚发的版本，看起来把死区从5改到了2.5，加了丢失后旋转方向的判断
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

@TeleOp(name = "LimelightTest")
public class LimelightTest extends LinearOpMode {
    private int latestTx;

    private static final int WINDOW_SIZE = 5;  // 高斯窗口大小
    private final Queue<Double> txHistory = new LinkedList<>();

    // 高斯权重
    private final double[] gaussianKernel = {0.06136, 0.24477, 0.38774, 0.24477, 0.06136};
    @Override
    public void runOpMode(){
        latestTx =0;
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "camera1");
        DcMotorEx cameraMotor = hardwareMap.get(DcMotorEx.class,"cameraMotor");
        limelight.pipelineSwitch(7);
        /*
         * Starts polling for data.
         */
        limelight.start();
        telemetry.update();

        cameraMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {
            List<LLResultTypes.FiducialResult> fiducialResult = limelight.getLatestResult().getFiducialResults();
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {

                    latestTx= (int) fiducial.getTargetXDegrees();

                    double txRaw = fiducial.getTargetXDegrees();


                    if (txHistory.size() >= WINDOW_SIZE) {
                        txHistory.poll();
                    }
                    txHistory.add(txRaw);

                    double txFiltered = applyGaussianFilter();

                    telemetry.addData("id",fiducial.getFiducialId());

                    double deadband = 2.5; //死区

                    if (Math.abs(txFiltered) > deadband) {

                        double kP = 0.02;
                        double power = kP * txFiltered;

                        power = Math.max(-0.1, Math.min(0.1, power));

                        cameraMotor.setPower(power);
                        telemetry.addData("MotorPower", power);
                    } else {
                        cameraMotor.setPower(0);
                        telemetry.addLine("Tag within range");
                    }
                }
            }
            else{
                if(latestTx>=0){
                    cameraMotor.setPower(0.1);
                }
                else{
                    cameraMotor.setPower(-0.1);
                }
                telemetry.addLine("finding tag");
            }
            telemetry.update();
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
