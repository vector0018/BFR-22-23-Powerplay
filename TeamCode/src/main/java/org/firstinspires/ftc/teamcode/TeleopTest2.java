package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
public class TeleopTest2 {
    @TeleOp

    public class Drive_test extends OpMode {
        DcMotor backleftmotor = null;
        DcMotor backrightmotor = null;
        DcMotor frontrightmotor = null;
        DcMotor frontleftmotor = null;
        DcMotor slideMotor = null;
        public double forwardMotorPower = gamepad1.left_stick_y * 0.7;
        public double turnPower = gamepad1.left_stick_x * 0.7;

        @Override
        public void init() {

        }

        @Override
        public void init_loop() {

        }

        @Override
        public void start() {

        }

        @Override
        public void loop() {
            backrightmotor.setPower(forwardMotorPower);
            frontrightmotor.setPower(forwardMotorPower);
            backleftmotor.setPower(forwardMotorPower);
            frontleftmotor.setPower(forwardMotorPower);

            if (gamepad1.left_stick_x > 0) {
                backleftmotor.setPower(turnPower);
                frontleftmotor.setPower(turnPower);
                backrightmotor.setPower(turnPower * -1.0);
                frontrightmotor.setPower(turnPower * -1.0);

                if (gamepad1.left_stick_x < 0) {
                    backleftmotor.setPower(turnPower * -1.0);
                    frontleftmotor.setPower(turnPower * -1.0);
                    backrightmotor.setPower(turnPower);
                    frontrightmotor.setPower(turnPower);
                }
            }
        }
                    @Override
                    public void stop (){
                        backleftmotor.setPower(0);
            }
        }
    }