package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Team_Close_Auto", group="ABC Opmode", preselectTeleOp = "DecodeTeleop")
public class Team_Close_Auto extends DecodeLibrary{
    public double forward = 0.25;
    public Path first_pick;
    public Path pick_after_stuff;
    public Path first_shoot;
    public Path second_shoot;
    public Path third_shoot;
    public Path fourth_shoot;
    public Path fifth_shoot;
    public Path second_pick;
    public Path third_pick;
    public Path park;
    public Path gate;
    public double shot = 0;
    public double old_turns = 0;
    public double steps = 0;
    public ElapsedTime gate_open = new ElapsedTime();
    public ElapsedTime shooting_time = new ElapsedTime();
    public double sort_pos = 0;
    public double old_pattern = 2;
    public double old_color = 1;
    public boolean shooting = false;
    public double index_steps = 0;
    public static double first_dump_time = 800;
    public static double second_dump_time = 800;
    public static double third_dump_time = 800;
    public static double auto_shoot_speed = 1400;
    @Override
    public void init(){
        color = 1;
        teleop = false;
        if(color == 1){
            auto_pose =new Pose(122,-25, Math.toRadians(-90));
        }else{
            auto_pose = new Pose(122,25, Math.toRadians(90));
        }
        initialize();
        follower.setPose(auto_pose);
        if(color == 0) {
            blue_init();
        }else{
            red_init();
        }
        follower.setMaxPower(1);
        follower.setMaxPowerScaling(1);
        follower.followPath(first_shoot);
        intake_system.flippy_pos = flippy_hold;
    }
    @Override
    public void init_loop(){
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        turret.zero = true;
        turret.turret_move();
        follower.drivetrain.setYVelocity(0);
        follower.update();
        if(gamepad1.right_bumper) {
            color = 0;
        }else if(gamepad1.left_bumper){
            color = 1;
        }
        if(old_color != color || gamepad1.a) {
            initialize();
            if(color == 1){
                auto_pose =new Pose(122,-25, Math.toRadians(-90));
            }else{
                auto_pose = new Pose(122,25, Math.toRadians(90));
            }
            follower.setMaxPower(1);
            follower.setMaxPowerScaling(1);
            follower.setPose(auto_pose);
            if (color == 0) {
                blue_init();
                old_color = 0;
            } else {
                red_init();
                old_color = 1;
            }

            follower.followPath(first_shoot);
        }
        auto_pose = follower.getPose();
        telemetry.addData("color", color);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void loop() {
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        follower.update();
        turret.turret_move();
        shooter.set_speed = shoot_multiplier * ((dead_distance * .0254) - 1.6) + shoot_power_offset + (speed_increase * abs(turret.target_angle - .5));
        shooter.set_speed = Math.round(shooter.set_speed / 20) * 20 + 40;
        shooter.shooting();
        intake_system.auto();
        auto_pose = follower.getPose();
        if(follower.atParametricEnd() || !follower.isBusy() || follower.isRobotStuck()) {
            if (forward == .25) {
                shoot();
                shooting = true;
                if(steps == 2){
                    shooting = false;
                    follower.followPath(first_pick);
                    forward = .5;
                    steps = 0;
                }
            }else if (forward == .5) {
                follower.followPath(gate);
                forward = .75;
            }else if(forward == .75){
                if(steps == 0){
                    gate_open.reset();
                    steps = 1;
                }else if(steps == 1 && gate_open.milliseconds() > first_dump_time) {
                    follower.followPath(second_shoot);
                    forward = .8;
                    steps = 0;
                }
            }else if (forward == .8) {
                shoot();
                shooting = true;
                if(steps == 2){
                    shooting = false;
                    follower.followPath(third_pick);
                    forward = 1;
                    steps = 0;
                }
            }else if (forward == 1) {
                follower.followPath(gate2);
                forward = 2;
            }else if(forward == 2){
                    if(steps == 0){
                        gate_open.reset();
                        steps = 1;
                    }else if(steps == 1 && gate_open.milliseconds() > second_dump_time) {
                        follower.followPath(fourth_shoot);
                        intake_system.outtake = true;
                        forward = 3;
                        steps = 0;
                    }
            }else if(forward == 3){
                intake_system.outtake = false;
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    follower.followPath(second_pick);
                    forward = 4;
                    steps = 0;
                }
            }else if(forward == 4) {
                follower.followPath(third_shoot);
                forward = 5;
                steps = 0;
            }else if(forward == 5){
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    follower.followPath(gate3);
                    forward = 6;
                    steps = 0;
                }
            }else if(forward == 6){
                if(steps == 0){
                    gate_open.reset();
                    steps = 1;
                }else if(steps == 1 && gate_open.milliseconds() > third_dump_time) {
                    follower.followPath(pick_after_stuff);
                    forward = 7;
                    steps = 0;
                }
            }else if(forward == 7){
                follower.followPath(fifth_shoot);
                shooting_time.reset();
                forward = 8;
            }else if(forward == 8 && shooting_time.milliseconds() > 1000){
                intake_system.flippy_pos = flippy_up;
                intake_system.blocking = false;
                forward = 9;
            }
        }
    }
    public Path gate2;
    public Path gate3;

    public void red_init(){
        first_shoot = new Path(new BezierLine(new Pose(123,-24), new Pose(78, -4)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
        first_pick = new Path(new BezierCurve(first_shoot.getLastControlPoint(), new Pose(49, -3), new Pose(47, -44)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        gate = new Path(new BezierCurve(first_pick.getLastControlPoint(), new Pose(52, -33), new Pose(53, -40.5)));
        gate.setConstantHeadingInterpolation(Math.toRadians(-90));
        second_shoot = new Path(new BezierCurve(gate.getLastControlPoint(),new Pose(53, -12), new Pose(72, -6)));
        second_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
        third_pick = new Path(new BezierCurve(second_shoot.getLastControlPoint(), new Pose(26, 0), new Pose(24, -41)));
        third_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        gate2 = new Path(new BezierCurve(third_pick.getLastControlPoint(), new Pose(47, -30), new Pose(49, -33), new Pose(53, -42)));
        gate2.setConstantHeadingInterpolation(Math.toRadians(-90));
        fourth_shoot = new Path(new BezierCurve(gate2.getLastControlPoint(),new Pose(53, -12), new Pose(72, -8)));
        fourth_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
        second_pick = new Path(new BezierCurve(fourth_shoot.getLastControlPoint(), new Pose(71, -36)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        third_shoot = new Path(new BezierLine(second_pick.getLastControlPoint(), new Pose(72, -10)));
        third_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
        gate3 = new Path(new BezierCurve(second_shoot.getLastControlPoint(), new Pose(58, -40)));
        gate3.setConstantHeadingInterpolation(Math.toRadians(-180));
        pick_after_stuff = new Path(new BezierCurve(gate2.getLastControlPoint(), new Pose(49,-41), new Pose(21, -49), new Pose(2, -49)));
        pick_after_stuff.setConstantHeadingInterpolation(Math.toRadians(-180));
        fifth_shoot = new Path(new BezierLine(pick_after_stuff.getLastControlPoint(), new Pose(90, -6)));
        fifth_shoot.setConstantHeadingInterpolation(Math.toRadians(-180));
        park = new Path(new BezierLine(fourth_shoot.getLastControlPoint(), new Pose(72,-15)));
        park.setConstantHeadingInterpolation(Math.toRadians(-45));
    }
    public void blue_init(){
        first_shoot = new Path(new BezierLine(new Pose(123,24), new Pose(78, 4)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        first_pick = new Path(new BezierCurve(first_shoot.getLastControlPoint(), new Pose(49, 3), new Pose(47, 44)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        gate = new Path(new BezierCurve(first_pick.getLastControlPoint(), new Pose(52, 33), new Pose(53, 40.5)));
        gate.setConstantHeadingInterpolation(Math.toRadians(90));
        second_shoot = new Path(new BezierCurve(gate.getLastControlPoint(),new Pose(53, 12), new Pose(72, 6)));
        second_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        third_pick = new Path(new BezierCurve(second_shoot.getLastControlPoint(), new Pose(26, 0), new Pose(24, 41)));
        third_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        gate2 = new Path(new BezierCurve(third_pick.getLastControlPoint(), new Pose(47, 30), new Pose(49, 33), new Pose(53, 42)));
        gate2.setConstantHeadingInterpolation(Math.toRadians(90));
        fourth_shoot = new Path(new BezierCurve(gate2.getLastControlPoint(),new Pose(53, 12), new Pose(72, 8)));
        fourth_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        second_pick = new Path(new BezierCurve(fourth_shoot.getLastControlPoint(), new Pose(71, 36)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        third_shoot = new Path(new BezierLine(second_pick.getLastControlPoint(), new Pose(72, 10)));
        third_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        gate3 = new Path(new BezierCurve(second_shoot.getLastControlPoint(), new Pose(58, 40)));
        gate3.setConstantHeadingInterpolation(Math.toRadians(180));
        pick_after_stuff = new Path(new BezierCurve(gate2.getLastControlPoint(), new Pose(49,41), new Pose(21, 49), new Pose(2, 49)));
        pick_after_stuff.setConstantHeadingInterpolation(Math.toRadians(180));
        fifth_shoot = new Path(new BezierLine(pick_after_stuff.getLastControlPoint(), new Pose(90, 6)));
        fifth_shoot.setConstantHeadingInterpolation(Math.toRadians(180));
        park = new Path(new BezierLine(fourth_shoot.getLastControlPoint(), new Pose(72,15)));
        park.setConstantHeadingInterpolation(Math.toRadians(45));
    }
    public void shoot(){
        if(shooting) {
            if(forward == .25 || forward > 5) {
                if (steps == 0 && abs(shooter.shoot1.getVelocity() - shooter.set_speed) <= 20) {
                    shooting_time.reset();
                    intake_system.flippy_pos = flippy_up;
                    intake_system.blocking = false;
                    if(forward > 5){
                        steps = .5;
                    }else {
                        steps = 1;
                    }
                }else if (steps == .5 && shooting_time.milliseconds() > 1000) {
                    steps = 1;
                    shooting_time.reset();
                }else if (steps == 1 && (shooting_time.milliseconds() > 1000 || shooter.balls_shot >=3)) {
                    intake_system.flippy_pos = flippy_down;
                    intake_system.spindexer.setPower(1);
                    intake_system.blocking = true;
                    steps = 2;
                }
            }else{
                intake_system.flippy_pos = flippy_down;
                if (steps == 0 && abs(shooter.shoot1.getVelocity() - shooter.set_speed) <= 20) {
                    shooting_time.reset();
                    intake_system.spindexer.setPower(-1);
                    intake_system.spindex_shooting = true;
                    intake_system.blocking = false;
                    steps = 1;
                } else if (steps == 1 && (shooting_time.milliseconds() > 3000 || shooter.balls_shot >=3)) {
                    if(forward == 5){
                        intake_system.flippy_pos = flippy_hold;
                        intake_system.spindexer.setPower(0);
                    }else {
                        intake_system.flippy_pos = flippy_down;
                        intake_system.spindexer.setPower(1);
                    }
                    intake_system.spindex_shooting = false;
                    intake_system.blocking = true;
                    steps = 2;
                }
            }
        }
    }



}
