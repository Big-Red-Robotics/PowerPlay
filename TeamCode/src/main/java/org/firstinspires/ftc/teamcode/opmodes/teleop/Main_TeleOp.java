package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Chassis;

@TeleOp(name="Main TeleOp :)")
public class Main_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        // init chassis
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        Chassis chassis = new Chassis(motorFL, motorFR, motorBL, motorBR, imu, telemetry);
        chassis.init();

        // init arms
        DcMotor leftLift = hardwareMap.get(DcMotor.class, "leftArm");
        DcMotor rightLift = hardwareMap.get(DcMotor.class, "rightArm");
        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        Servo cam = hardwareMap.get(Servo.class, "cam");
        Arm arm = new Arm(leftLift, rightLift, gripper, cam);
        arm.init();
        arm.armTarget = 0;

        int stackHeight = arm.ground;

        boolean stack = false;

        telemetry.addData("cam position", cam.getPosition());

        waitForStart();

        while (opModeIsActive()) {
            chassis.joyStick(gamepad1);

            if (gamepad2.x) { arm.armTarget = arm.lowJunction; stack = false; }
            if (gamepad2.b) { arm.armTarget = arm.middleJunction; stack = false; }
            if (gamepad2.y) { arm.armTarget = arm.highJunction; stack = false; }
            if (gamepad2.a) { arm.armTarget = 0; stackHeight = arm.ground; stack = false; }

            if (gamepad2.dpad_up) { arm.armTarget = 250; stackHeight = arm.fiveStack; stack = true; }
            if (gamepad2.dpad_left) { arm.armTarget = 250; stackHeight = arm.fourStack; stack = true; }
            if (gamepad2.dpad_right) { arm.armTarget = 250; stackHeight = arm.threeStack; stack = true; }
            if (gamepad2.dpad_down) { arm.armTarget = 250; stackHeight = arm.twoStack; stack = true; }

            if (gamepad2.left_bumper){
                arm.armTarget = 0;
                stackHeight = arm.ground;
                stack = false;
                arm.openGripper();
            }
            if (gamepad2.right_bumper) arm.closeGripper();


            //TODO: SET 1.0 FOR 11166-RC!!
            if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
                stack = false;
                arm.armTriggers(gamepad2);
            } else {
                arm.setArmPower(gamepad2, 1.0, stackHeight, stack);
            }

            telemetry.addData("arm position", arm.getCurrentPosition());
            telemetry.addData("arm target", arm.armTarget);
            telemetry.addData("Servo position", arm.gripper.getPosition());
            telemetry.addData("Cam:",arm.cam.getPosition());
            telemetry.update();
        }
    }
}
