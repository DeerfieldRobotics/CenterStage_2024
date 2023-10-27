package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain
{
    // initialize motors, hardwareMap
    private DcMotor fl, fr, bl, br;
    private HardwareMap hw;

    //constants
    static final double     COUNTS_PER_MOTOR_REV    = 1 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DRIVE_SPEED = 0.6;
    public static final double TURN_SPEED = 0.5;

    private double drivetrainPowMod = 0.5;


    public Drivetrain (HardwareMap hardwaremap) {
        hw = hardwaremap;

        initialize();
    }

    public void initialize() {
        fr = hw.get(DcMotor.class, "fr");
        fl = hw.get(DcMotor.class, "fl");
        br = hw.get(DcMotor.class, "br");
        bl = hw.get(DcMotor.class, "bl");

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void floatMotors() {

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    /**
     * Moves drivetrain
     *
     * @param strafe speed of strafe between 0 and 1
     * @param turn speed of turn between 0 and 1
     * @param forward speed of forward between 0 and 1
     */
    public void move(double strafe, double turn, double forward)
    {
        turn *= -drivetrainPowMod;
        forward *= drivetrainPowMod;
        strafe *= -drivetrainPowMod;
        fl.setPower(forward + turn + strafe);
        fr.setPower(forward - turn - strafe);
        bl.setPower(forward + turn - strafe);
        br.setPower(forward - turn + strafe);
        stop();
    }

    public void setMotorPowers(double flpow, double frpow, double blpow, double brpow){
        fl.setPower(flpow);
        fr.setPower(frpow);
        bl.setPower(blpow);
        br.setPower(brpow);
    }
    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

    }

    public void stop_reset_encoder(){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void run_to_pos(){
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void no_encoder(){
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void run_using_encoder(){
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setDrivetrainSpeedMod(double speed) {
        drivetrainPowMod=speed;
    }

    public int[] getMotorPositions()
    {
        return new int[]
                {fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition()};
    }

    public void setPosition(int fl, int fr, int bl, int br){
        this.fl.setTargetPosition(fl);
        this.fr.setTargetPosition(fr);
        this.bl.setTargetPosition(bl);
        this.br.setTargetPosition(br);
    }

    public String printMotorPowers(){
        return "fl: " + fl.getPower() +"fr: " + fr.getPower()+"bl: " + bl.getPower()+"br: " + br.getPower();
    }

    public void forwards(boolean backwards, int LTarget, int RTarget, double speed_mod)
    {
        double pow = drivetrainPowMod * speed_mod;
        int mult = 1;
        if(!backwards) {
            //backwards
            mult *= -1;
        }
        int FLTarget = mult * (int) (LTarget * 31);
        int FRTarget = mult * (int) (RTarget * 31);
        int BLTarget = mult  *(int) (LTarget * 31);
        int BRTarget = mult * (int) (RTarget * 31);


        setPosition(FLTarget, FRTarget, BLTarget, BRTarget);

        run_to_pos();

        setMotorPowers(pow, pow, pow, pow);
    }

    public void strafe(boolean left, int LTarget, int RTarget)
    {
        double pow= drivetrainPowMod;
        int mult = 1;
        if (!left) {
            //right
            mult =-1;
        }

        int FLTarget = mult * (int) (LTarget * 31);
        int FRTarget = -mult * (int) (RTarget * 31);
        int BLTarget = -mult *(int) (LTarget * 31);
        int BRTarget = mult * (int) (RTarget * 31);

        setPosition(FLTarget, FRTarget, BLTarget, BRTarget);

        run_to_pos();

        setMotorPowers(pow, pow, pow,pow);

    }

    public void turn(boolean left, int amount) {
        if (!left) {
            fr.setTargetPosition(-amount);
            fl.setTargetPosition(amount);
            br.setTargetPosition(-amount);
            bl.setTargetPosition(amount);
        } else {
            fr.setTargetPosition(amount);
            fl.setTargetPosition(-amount);
            br.setTargetPosition(amount);
            bl.setTargetPosition(-amount);
        }

        run_to_pos();
    }

    public boolean isBusy(){
        return fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy();
    }

}