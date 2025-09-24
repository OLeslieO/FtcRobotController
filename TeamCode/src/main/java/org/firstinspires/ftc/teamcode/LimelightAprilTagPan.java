package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

@TeleOp(name="LimelightAprilTagPan")
public class LimelightAprilTagPan extends OpMode {

    private Servo panServo;

    // 舵机初始位置
    private final double panCenter = 0.5;
    private double panPos = panCenter;

    // 调节系数，可在 loop 中调试
    private final double kPan = 0.005;

    @Override
    public void init() {
        panServo = hardwareMap.get(Servo.class, "panServo");
        panServo.setPosition(panCenter);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        double tx = getLimelightTX();  // 获取 Limelight 水平偏移

        // 根据偏差调整舵机位置
        panPos -= tx * kPan;

        // 限制舵机在 0 ~ 1 范围
        panPos = Math.max(0.0, Math.min(1.0, panPos));

        // 设置舵机
        panServo.setPosition(panPos);

        // 调试信息
        telemetry.addData("tx", tx);
        telemetry.addData("Pan Pos", panPos);
        telemetry.update();
    }

    /**
     * 获取 Limelight 水平偏移角 tx
     * @return tx，单位为度。目标在中间返回 0。
     */
    private double getLimelightTX() {
        // 获取 Limelight NetworkTable
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry txEntry = limelightTable.getEntry("tx");

        // 如果没有目标，返回 0.0
        return txEntry.getDouble(0.0);
    }
}
