package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Limelight Debug")
public class LimelightDebug extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx cameraMotor = hardwareMap.get(DcMotorEx.class, "cameraMotor");


        cameraMotor.setTargetPosition(0);
        cameraMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        limelight = hardwareMap.get(Limelight3A.class, "camera1"); // 去掉空格

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(7); // 确认 pipeline 0 是 AprilTag 模式
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            

            if (result != null && result.isValid()) {
                for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    telemetry.addData("TagID", tag.getFiducialId());
                    telemetry.addData("tx", tag.getTargetXDegrees());
                    telemetry.addData("ty", tag.getTargetYDegrees());
                    double tx = tag.getTargetXDegrees();

                    if (tx < 0) {
                        cameraMotor.setPower(-0.1);
                    } else if (tx > 0) {
                        cameraMotor.setPower(0.1);
                    } else {
                        cameraMotor.setPower(0);
                    }
                }
            } else {
                telemetry.addLine("No tag detected");
                cameraMotor.setPower(0.1);
            }
            telemetry.update();
        }
    }
}
