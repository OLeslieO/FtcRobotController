package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Vision {
    private final Limelight3A camera; //limelight相机
    private final Servo gimbalServo;  // 云台舵机

    private double servoPos = 0.5;    // 舵机初始位置（0~1）

    Telemetry telemetry;

    public Vision(@NonNull final HardwareMap hardwareMap, Telemetry telemetry) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        gimbalServo = hardwareMap.get(Servo.class, "gimbal"); // 硬件映射：云台舵机

        this.telemetry = telemetry;
    }

    public void initialize() {
        gimbalServo.setPosition(servoPos);
        camera.setPollRateHz(50);
        camera.start();
    }

    public LLResult getResult() {
        return camera.getLatestResult();
    }

    public boolean resultValid(@NonNull LLResult result){
        return result.getTa() != 0 && result.getStaleness() < 30;
    }

    /***
     * 云台自动对准 AprilTag
     */
    public void trackAprilTag() {
        LLResult result = getResult();
        if(result != null && resultValid(result)) {
            double tx = result.getTx(); //左负右正
            // 比例系数，用于控制灵敏度
            double kP = 0.01;
            double correction = kP * tx; // P 控制器输出

            servoPos -= correction; // 偏差左负右正，修正方向相反
            double minPos = 0.05;
            double maxPos = 0.95;
            servoPos = Math.max(minPos, Math.min(maxPos, servoPos)); // 限制范围
            gimbalServo.setPosition(servoPos);

            telemetry.addData("tx", tx);
            telemetry.addData("servoPos", servoPos);
        } else {
            telemetry.addData("AprilTag", "Not Found");
        }
    }

    public void update(boolean debugMode){
        trackAprilTag();  // 更新时自动追踪AprilTag

        if(debugMode){
            telemetry.update();
        }
    }
}
