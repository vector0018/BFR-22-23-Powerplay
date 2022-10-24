package org.firstinspires.ftc.teamcode.teleop.teleoptests;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp
public class WheelTest extends OpMode {
    DcMotor leftFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    DcMotor rightFront = null;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        leftRear = hardwareMap.get(DcMotorEx.class, "LB");
        rightRear = hardwareMap.get(DcMotorEx.class, "RB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            leftFront.setPower(0.5);
        } else {
            leftFront.setPower(0);
        }

        if (gamepad1.b) {
            leftRear.setPower(0.5);
        } else {
            leftRear.setPower(0);
        }

        if (gamepad1.x) {
            rightFront.setPower(0.5);
        } else {
            rightFront.setPower(0);
        }

        if (gamepad1.y) {
            rightRear.setPower(0.5);
        } else {
            rightRear.setPower(0);
        }

        if (gamepad1.right_bumper) {
            rightFront.setPower(-0.5);
        } else {
            rightFront.setPower(0);
        }
    }
}