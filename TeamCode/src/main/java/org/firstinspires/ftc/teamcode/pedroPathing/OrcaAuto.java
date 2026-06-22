package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Arrays;
import java.util.List;
@Config

public class OrcaAuto {
    public List<DcMotorEx> motors;
    public GoBildaPinpointDriver pinpoint;
    public List<Pose2D> currentPath = null;
    public void SwerveOrcaInit(HardwareMap hardwareMap, Gamepad gamepad, double color1) {
        forward_pid = new PIDController(forward_pidf.p, forward_pidf.i,forward_pidf.d);
        strafe_pid = new PIDController(strafe_pidf.p, strafe_pidf.i,strafe_pidf.d);
        auto_drive = false;
        color = color1;
        gamepad1 = gamepad;
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        double forwardPodY = 4.25;
        double strafePodX = -.5;
        pinpoint.setOffsets(forwardPodY, strafePodX, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        chassis.initialize(hardwareMap);
        motors = Arrays.asList(chassis.fr.wheel, chassis.fl.wheel, chassis.rr.wheel, chassis.rl.wheel);
    }
    public swerve_drive chassis = new swerve_drive();
    public boolean auto_drive = false;
    public boolean x_out = false;
    public boolean zero = false;
    public boolean stop_and_point = false;
    public double angle = 0;
    public Gamepad gamepad1;
    public double color = 0;
    public static double fr_offset = -1130;
    public static double fl_offset = 720;
    public static double rr_offset = -1660;
    public static double rl_offset = 80;
    public static double p = 0.00055, i = 0, d = .0000001 ;

    public static double f = 0.005;
    public static double degree_offset = 0;
    public static double change = 150;
    public static double angle_offset = 360;
    public PIDController forward_pid;
    public PIDController strafe_pid;
    public static PIDFCoefficients forward_pidf = new PIDFCoefficients(.02, 0.09, 1, .035);
    public static PIDFCoefficients strafe_pidf = new PIDFCoefficients(.005, 0.05, 1, 0);
    public double targetDistance = 0;
    public static double target_threshold = 6;
    public double speed = 0;
    public double real_angle = 0;
    public Pose2D position = new Pose2D(DistanceUnit.INCH, 0,0,AngleUnit.DEGREES,0);
    public boolean was_turning = true;
    public double keep_angle = 0;
    public boolean hold_angle = false;
    public double angle_to_hold = 0;
    public void update_pid(){
        strafe_pid = new PIDController(.01, strafe_pidf.i,strafe_pidf.d);
    }


    public void runDrive() {
        pinpoint.update();
        position = pinpoint.getPosition();
        if(x_out){
            chassis.x();
        }else if(zero){
            chassis.zero();
        }else {
            angle = position.getHeading(AngleUnit.DEGREES);
            real_angle = angle;
            if(real_angle < 0){
                real_angle = abs(real_angle);
            }else{
                real_angle = angle_offset - real_angle;
            }
            if(auto_drive){
                real_angle += 90;
            }else {
                if (color == 0) {
                    real_angle += 90;
                } else {
                    real_angle -= 90;
                }
            }
            if(real_angle > 360){
                real_angle -= 360;
            }else if(real_angle < 0){
                real_angle += 360;
            }
            real_angle = Math.toRadians(real_angle);
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double strafe = gamepad1.right_stick_x;
            if(auto_drive){
                if(currentPath == null || currentPath.isEmpty()){
                    x = 0;
                    y = 0;
                    strafe = 0;
                }else{
                    if(currentPath.size() == 1){
                        forward_pid.setPID(.04, forward_pidf.i,forward_pidf.d);
                    }else{
                        forward_pid.setPID(.05, forward_pidf.i,forward_pidf.d);
                    }
                    double new_x = currentPath.get(0).getX(DistanceUnit.INCH) - position.getX(DistanceUnit.INCH);
                    double new_y = position.getY(DistanceUnit.INCH) - currentPath.get(0).getY(DistanceUnit.INCH);
                    if(new_x == 0){
                        new_x = .00001;
                    }
                    if(new_y == 0){
                        new_y = .00001;
                    }
                    y = sin(abs(atan(new_y/new_x))) * signum(new_y);
                    x = cos(abs(atan(new_y/new_x))) * signum(new_x);
                    targetDistance = sqrt(new_y*new_y + new_x*new_x);
                    double velocity = (pinpoint.getVelX(DistanceUnit.INCH) + pinpoint.getVelY(DistanceUnit.INCH)) / 2;
                    if(targetDistance < target_threshold && (currentPath.size() > 1 || (targetDistance < 3))){
                        currentPath.remove(0);
                        if(currentPath.isEmpty()) {
                            currentPath = null;
                        }
                    }
                    if(currentPath !=  null) {
                        double pid = forward_pid.calculate(0, targetDistance);
                        double ff = cos(Math.toRadians(targetDistance)) * forward_pidf.f;
                        double power = abs(pid + ff);
                        if (power > 1) {
                            power = 1;
                        }
                        y *= power;
                        x *= power;
                        double targetAngle = currentPath.get(0).getHeading(AngleUnit.DEGREES) - Math.toDegrees(real_angle);
                        if (targetAngle > 180) {
                            targetAngle -= 360;
                        } else if (targetAngle < -180) {
                            targetAngle += 360;
                        }
                        double spid = strafe_pid.calculate(0, targetAngle);
                        double sff = cos(Math.toRadians(targetAngle)) * strafe_pidf.f;
                        strafe = spid + sff;
                        if (strafe > 1) {
                            strafe = 1;
                        }
                    }
                }


            }else{
                if(abs(x) < .05 && abs(y) < .05 && abs(strafe) < .05){
                    zero = true;
                }
                if(hold_angle){
                    double targetAngle = angle_to_hold - Math.toDegrees(real_angle);
                    if (targetAngle > 180) {
                        targetAngle -= 360;
                    } else if (targetAngle < -180) {
                        targetAngle += 360;
                    }
                    double spid = strafe_pid.calculate(0, targetAngle);
                    double sff = cos(Math.toRadians(targetAngle)) * strafe_pidf.f;
                    strafe = spid + sff;
                    if (strafe > 1) {
                        strafe = 1;
                    }
                }
            }
            double fwd = -y * cos(real_angle) + (x) * sin(real_angle);
            double str = (x) * cos(real_angle) + y * sin(real_angle);
            if(zero){
                chassis.zero();
            }else {
                chassis.drive(fwd, str, strafe);
            }
        }
        x_out = false;
        stop_and_point = false;
        zero = false;
    }

    public class swerve_drive {
        double L = 0;
        double W = 0;
        double R = 0;
        double A = 0;
        double B = 0;
        double C = 0;
        double D = 0;
        double frs = 0;
        double fls = 0;
        double rrs = 0;
        double rls = 0;
        double fra = 0;
        double fla = 0;
        double rra = 0;
        double rla = 0;
        public swerve_drive.swerve_pod fr = new swerve_drive.swerve_pod();
        public swerve_drive.swerve_pod fl = new swerve_drive.swerve_pod();
        public swerve_drive.swerve_pod rr = new swerve_drive.swerve_pod();
        public swerve_drive.swerve_pod rl = new swerve_drive.swerve_pod();

        public void initialize(HardwareMap hardwareMap) {
            fr.initialize(hardwareMap, "fr_motor", "fr_servo", "fr_analog");
            fl.initialize(hardwareMap, "fl_motor", "fl_servo", "fl_analog");
            rr.initialize(hardwareMap, "rr_motor", "rr_servo", "rr_analog");
            rl.initialize(hardwareMap, "rl_motor", "rl_servo", "rl_analog");
        }
        public class swerve_pod {

            public PIDController controller;

            public double position = 0;
            public CRServo pod1servo = null;
            public DcMotorEx wheel = null;
            public AnalogInput pod1rotation = null;
            public ElapsedTime time = new ElapsedTime();
            public double wheel_pos;
            public double target;
            public double speed = 0;
            public double power = 0;
            public double wheel_power = 0;

            public void initialize(HardwareMap hardwareMap, String wheel_name, String servo_name, String analog_name) {
                controller = new PIDController(p, i, d);
                wheel = hardwareMap.get(DcMotorEx.class, wheel_name);
                wheel.setDirection(DcMotorSimple.Direction.REVERSE);
                wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                pod1servo = hardwareMap.get(CRServo.class, servo_name);
                pod1rotation = hardwareMap.get(AnalogInput.class, analog_name);
                time.reset();
            }

            public void move(double offset, double inversion) {
                wheel_pos = (abs(inversion - pod1rotation.getVoltage() / 3.3) * 3600) + offset;
                if (wheel_pos < 0) {
                    wheel_pos += 3600;
                }
                if (wheel_pos > 3600) {
                    wheel_pos -= 3600;
                }
                target = position * 10;
                if (abs(target - wheel_pos) > 900 && abs(target - wheel_pos) < 2700) {
                    target -= 1800;
                    wheel_power *= -1;
                }
                if (target < 0) {
                    target += 3600;
                }
                if (target - wheel_pos > 1800) {
                    target -= 3600;
                } else if (target - wheel_pos < -1800) {
                    target += 3600;
                }
                controller.setPID(p, i, d);
                double pid = controller.calculate(wheel_pos, target);
                double ff = cos(Math.toRadians(target)) * f;
                power = pid + ff;

                if (inversion == 1) {
                    pod1servo.setPower(-power);
                } else {
                    pod1servo.setPower(power);
                }
                if (!auto_drive) {
                    wheel.setPower(wheel_power);
                } else {
                    wheel.setPower(wheel_power / (abs(target - wheel_pos) / change));
                }

            }
        }
        public void zero(){
            fr.position = 0;
            fr.wheel_power = 0;
            fl.position = 0;
            fl.wheel_power = 0;
            rr.position = 0;
            rr.wheel_power = 0;
            rl.position = 0;
            rl.wheel_power = 0;
            fr.move(fr_offset,1);
            fl.move(fl_offset,1);
            rr.move(rr_offset,1);
            rl.move(rl_offset,1);

        }
        public void x(){
            double wheel_angle = angle;
            fr.position = wheel_angle;
            fr.wheel_power = 0;
            fl.position = wheel_angle;
            fl.wheel_power = 0;
            rr.position = wheel_angle;
            rr.wheel_power = 0;
            rl.position = wheel_angle;
            rl.wheel_power = 0;
            fr.move(fr_offset,1);
            fl.move(fl_offset,1);
            rr.move(rr_offset,1);
            rl.move(rl_offset,1);

        }
        public void drive(double fwd, double str, double yaw) {
            L = 9.625;
            W = 13.375;
            R = Math.sqrt(L * L + W * W);
            A = str - yaw * L / R;
            B = str + yaw * L / R;
            C = fwd - yaw * W / R;
            D = fwd + yaw * W / R;
            frs = Math.sqrt(B * B + C * C);
            fls = Math.sqrt(B * B + D * D);
            rrs = Math.sqrt(A * A + C * C);
            rls = Math.sqrt(A * A + D * D);
            fra = Math.toDegrees(Math.atan2(B, C)) + degree_offset;
            fla = Math.toDegrees(Math.atan2(B, D)) + degree_offset;
            rra = Math.toDegrees(Math.atan2(A, C)) + degree_offset;
            rla = Math.toDegrees(Math.atan2(A, D)) + degree_offset;
            double max = Math.max(fls, frs);
            max = Math.max(max, rrs);
            max = Math.max(max, rls);
            if (max > 1) {
                fls = fls / max;
                frs = frs / max;
                rrs = rrs / max;
                rls = rls / max;
            }
            if (fra < 0) {
                fra += 360;
            }
            if (fla < 0) {
                fla += 360;
            }
            if (rra < 0) {
                rra += 360;
            }
            if (rla < 0) {
                rla += 360;
            }

            fr.position = fra;
            fr.wheel_power = frs;
            fl.position = fla;
            fl.wheel_power = fls;
            rr.position = rra;
            rr.wheel_power = rrs;
            rl.position = rla;
            rl.wheel_power = rls;
            if(stop_and_point){
                fr.wheel_power = 0;
                fl.wheel_power = 0;
                rr.wheel_power = 0;
                rl.wheel_power = 0;
            }
            fr.move(fr_offset, 1);
            fl.move(fl_offset, 1);
            rr.move(rr_offset, 1);
            rl.move(rl_offset, 1);

        }

    }
}
