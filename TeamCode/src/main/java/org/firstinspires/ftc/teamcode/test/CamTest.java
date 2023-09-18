package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Chassis;

public class CamTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo cam = hardwareMap.get(Servo.class, "cam");

        telemetry.addLine("waiting to start!");
        telemetry.update();

        cam.setPosition(0.508);
        waitForStart();

        while(opModeIsActive()) {
            if (gamepad2.dpad_up) { cam.setPosition(0.526); }
            if (gamepad2.dpad_left) { cam.setPosition(0.53); }
            if (gamepad2.dpad_right) { cam.setPosition(0.536); }
            if (gamepad2.dpad_down) { cam.setPosition(0.542); }
            telemetry.addData("cam", cam.getPosition());
            telemetry.update();
        }
    }
}