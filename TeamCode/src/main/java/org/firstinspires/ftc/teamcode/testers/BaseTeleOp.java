package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="BaseTeleOp", group="TeleOp")
public class BaseTeleOp extends LinearOpMode {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private double speed;
    private double[] oldVelos;

    final double HIGH_SPEED = 1.0;
    final double LOW_SPEED = 0.7;
    final double PRECISE_SPEED = 0.2;

    final double TURN_SPEED = 1.0;
    
    // Acceleration Per Tick Limit
    final double APT_LIMIT = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right) { 
                preciseset();
            } else {
                driveset();
            }
        }
    }
    
    private void driveset() {
        if (gamepad2.right_bumper) {
            speed = HIGH_SPEED;
        }
        if (gamepad2.left_bumper) {
            speed = LOW_SPEED;
        }
        
        double ls_y = -gamepad2.left_stick_y;
        double ls_x = -gamepad2.left_stick_x;
        double rs_x = -gamepad2.right_stick_x;

        //                   move  strafe    turn
        double frontLeftV  = ls_x - ls_y + TURN_SPEED * rs_x;
        double frontRightV = ls_x + ls_y + TURN_SPEED * rs_x;
        double backLeftV   = ls_x + ls_y - TURN_SPEED * rs_x;
        double backRightV  = ls_x - ls_y - TURN_SPEED * rs_x;

        // shrinking speed's range from [-(2.0 + TURN_SPEED), +(2.0 + TURN_SPEED)] to [-1.0, +1.0]
        // then multiplying it by "speed" to make higher / lower
        frontLeftV  = frontLeftV  / (2 + TURN_SPEED) * speed;
        frontRightV = frontRightV / (2 + TURN_SPEED) * speed;
        backLeftV   = backLeftV   / (2 + TURN_SPEED) * speed;
        backRightV  = backRightV  / (2 + TURN_SPEED) * speed;

        double[] accelerationV = new double[4];
        accelerationV[0] = frontLeftV - oldVelos[0];
        accelerationV[1] = frontRightV - oldVelos[1];
        accelerationV[2] = backLeftV - oldVelos[2];
        accelerationV[3] = backRightV - oldVelos[3];

        // acceleration's absolute value can't be bigger than acceleration limit
        // a =             if a is positive   then               this               else                this
        accelerationV[0] = accelerationV[0] > 0 ? Math.min(APT_LIMIT, accelerationV[0]) : Math.max(-APT_LIMIT, accelerationV[0]);
        accelerationV[1] = accelerationV[1] > 0 ? Math.min(APT_LIMIT, accelerationV[1]) : Math.max(-APT_LIMIT, accelerationV[1]);
        accelerationV[2] = accelerationV[2] > 0 ? Math.min(APT_LIMIT, accelerationV[2]) : Math.max(-APT_LIMIT, accelerationV[2]);
        accelerationV[3] = accelerationV[3] > 0 ? Math.min(APT_LIMIT, accelerationV[3]) : Math.max(-APT_LIMIT, accelerationV[3]);

        // set new speed
        frontLeft.setPower(oldVelos[0] + accelerationV[0]);
        frontRight.setPower(oldVelos[1] + accelerationV[1]);
        backLeft.setPower(oldVelos[2] + accelerationV[2]);
        backRight.setPower(oldVelos[3] + accelerationV[3]);

        // save new speed
        oldVelos[0] += accelerationV[0];
        oldVelos[1] += accelerationV[1];
        oldVelos[2] += accelerationV[2];
        oldVelos[3] += accelerationV[3];
    }

    private void preciseset() {
        // set speed
        if (gamepad2.dpad_up) { // move forward
            frontLeft.setPower(-PRECISE_SPEED);
            frontRight.setPower(PRECISE_SPEED);
            backLeft.setPower(PRECISE_SPEED);
            backRight.setPower(-PRECISE_SPEED);
        } 
        else if (gamepad2.dpad_down) { // move backward
            frontLeft.setPower(PRECISE_SPEED);
            frontRight.setPower(-PRECISE_SPEED);
            backLeft.setPower(-PRECISE_SPEED);
            backRight.setPower(PRECISE_SPEED);
        } 
        else if (gamepad2.dpad_left) { // strafe left
        frontLeft.setPower(PRECISE_SPEED);
            frontRight.setPower(PRECISE_SPEED);
            backLeft.setPower(PRECISE_SPEED);
            backRight.setPower(PRECISE_SPEED);
        } 
        else if (gamepad2.dpad_right) { // strafe right
            frontLeft.setPower(-PRECISE_SPEED);
            frontRight.setPower(-PRECISE_SPEED);
            backLeft.setPower(-PRECISE_SPEED);
            backRight.setPower(-PRECISE_SPEED);
        }
    }

    public void initialize() {
        // set initial speed
        speed = HIGH_SPEED;
        oldVelos = new double[4];

        // initialize DCMotors
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        backLeft = hardwareMap.get(DcMotor.class, "bl");

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}