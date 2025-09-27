package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name="CameraTest1")
public class Test1 extends LinearOpMode {
    //    private Limelight3A cameraFront;
    private static final int TARGET_TAG_ID = 23;

    private DcMotorEx cameraMotor;



    @Override
    public void runOpMode() throws InterruptedException {



        //      初始化硬件路径
//        cameraFront= hardwareMap.get(Limelight3A.class, "camera1");

        cameraMotor=hardwareMap.get(DcMotorEx.class, "cameraMotor");
//初始化odo

//        初始化电机
        cameraMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        cameraMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cameraMotor.setTargetPosition(0);
        cameraMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cameraMotor.setPower(0.01);
        // Declare our motors

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "camera1");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(7);
        limelight.start();


        waitForStart();
        while (opModeIsActive()){


            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                // 遍历所有检测到的 AprilTag
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    if (tag.getFiducialId() == TARGET_TAG_ID) {  // 只处理指定 ID
                        double tx = result.getTx();  // 左右偏移角度

                        telemetry.addData("检测到目标 ID", tag.getFiducialId());
                        telemetry.addData("目标 X", tx);
                        telemetry.update();


                        if (tx < 0) {
                            cameraMotor.setPower(-0.2);
                        } else if (tx > 0) {
                            cameraMotor.setPower(0.2);
                        } else {
                            cameraMotor.setPower(0);
                        }
                    }
                }
            } else {
                telemetry.addData("Limelight", "没有目标");
                cameraMotor.setPower(0.01);  // 丢失目标时停止
                telemetry.update();
            }





    }
    }}

