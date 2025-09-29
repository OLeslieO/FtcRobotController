package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Jack MecanumNew")
public class mmmm1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFrontMotor=hardwareMap.get(DcMotorEx.class,"leftFrontMotor");
        DcMotorEx leftBackMotor=hardwareMap.get(DcMotorEx.class,"leftBackMotor");
        DcMotorEx rightFrontMotor=hardwareMap.get(DcMotorEx.class,"rightFrontMotor");
        DcMotorEx rightBackMotor=hardwareMap.get(DcMotorEx.class,"rightBackMotor");


        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT ));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        waitForStart();
        while (opModeIsActive()) {
            double y=-gamepad1.left_stick_y;
            double x= gamepad1.left_stick_x;
            double rx= gamepad1.right_stick_x;
            if (gamepad1.options) {
                imu.resetYaw();
            }
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            //rotX = rotX * 1.1;  // Counteract imperfect strafing
            double dom=Math.max(Math.abs(rotX)+Math.abs(rotY)+Math.abs(rx),1);
            leftFrontMotor.setPower((rotY + rotX + rx)/dom);
            leftBackMotor.setPower((rotY - rotX + rx)/dom);
            rightFrontMotor.setPower((rotY - rotX - rx)/dom);
            rightBackMotor.setPower((rotY + rotX - rx)/dom);
        }
    }
}