package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleopTest2 extends OpMode {
        DcMotor backLeftMotor = null;
        DcMotor backRightMotor = null;
        DcMotor frontRightMotor = null;
        DcMotor frontLeftMotor = null;
        DcMotor slideMotor = null;
        public double forwardMotorPower = gamepad1.left_stick_y * 0.7;
        public double turnPower = gamepad1.left_stick_x * 0.7;

        @Override
        public void init() {
            backLeftMotor = hardwareMap.get(DcMotor.class,"LB");
            backRightMotor = hardwareMap.get(DcMotor.class,"RB");
            frontRightMotor = hardwareMap.get(DcMotor.class,"RF");
            frontLeftMotor = hardwareMap.get(DcMotor.class,"LF");

            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
            backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        @Override
        public void init_loop() {

        }

        @Override
        public void start() {

        }

        @Override
        public void loop() {
            backRightMotor.setPower(forwardMotorPower);
            frontRightMotor.setPower(forwardMotorPower);
            backLeftMotor.setPower(forwardMotorPower);
            frontLeftMotor.setPower(forwardMotorPower);

            if (gamepad1.left_stick_x > 0) {
                backLeftMotor.setPower(turnPower);
                frontLeftMotor.setPower(turnPower);
                backRightMotor.setPower(turnPower * -1.0);
                frontRightMotor.setPower(turnPower * -1.0);

                if (gamepad1.left_stick_x < 0) {
                    backLeftMotor.setPower(turnPower * -1.0);
                    frontLeftMotor.setPower(turnPower * -1.0);
                    backRightMotor.setPower(turnPower);
                    frontRightMotor.setPower(turnPower);
                }
            }
        }
                    @Override
                    public void stop (){
                        backLeftMotor.setPower(0);
                        frontLeftMotor.setPower(0);
                        backRightMotor.setPower(0);
                        frontRightMotor.setPower(0);
            }
}
