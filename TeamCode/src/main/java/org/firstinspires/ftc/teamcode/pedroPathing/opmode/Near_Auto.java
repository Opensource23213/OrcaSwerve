package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Near_Auto", group="ABC Opmode", preselectTeleOp = "DecodeTeleop")
public class Near_Auto extends DecodeLibrary{
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
    public boolean re_init = false;
    public Path nextPath = new Path();
    public boolean second_dump = false;
    @Override
    public void init(){
        color = 1;
        teleop = false;
        initialize();
        if(color == 0) {
            follower.setPose(new Pose(122,25 - .375, Math.toRadians(180)));
            blue_init();
        }else{
            follower.setPose(new Pose(122,-25, Math.toRadians(-180)));
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
       /* turret.zero = true;
        turret.manual_angle = -90;
        turret.turret_move();*/
        follower.drivetrain.setYVelocity(0);
        follower.update();
        if(gamepad1.a){
            color = 1;
            re_init = true;
        }else if(gamepad1.b){
            color = 0;
            re_init = true;
        }

        if(old_color != color || re_init) {
            re_init = false;
            initialize();
            if (color == 0) {
                follower.setPose(new Pose(122,25 - .375, Math.toRadians(180)));
                blue_init();
                old_color = 0;
            } else {
                follower.setPose(new Pose(122,-24, Math.toRadians(-180)));
                red_init();
                old_color = 1;
            }
            follower.followPath(first_shoot);
        }
    }

    @Override
    public void loop() {
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        /*turret.zero = true;
        turret.manual_angle = -90;
        turret.turret_move();
        shooter.set_speed = 1200;
        shooter.shooting();
        intake_system.auto();*/
        follower.update();
        auto_pose = follower.getPose();
        if(follower.atParametricEnd() || !follower.isBusy() || follower.isRobotStuck() || steps == 2 || (follower.getCurrentPath() == gate && abs(follower.getPose().getY()) > abs(gate.getLastControlPoint().getY()) - error)) {
            if(forward == .25){
                shooting = true;
                nextPath = first_pick;
                shoot();
                if(steps == 2) {
                    shooting = false;
                    forward = .5;
                    steps = 0;
                }
            }
            else if(forward == .5){
                follower.followPath(second_shoot);
                forward = 1;
            }
            else  if(forward == 1){
                shooting = true;
                nextPath = second_pick;
                shoot();
                if(steps == 2) {
                    shooting = false;
                    follower.followPath(second_pick);
                    forward = 2;
                    steps = 0;
                }
            }
            else if(forward == 2){
                follower.followPath(third_shoot);
                forward = 3;
            }else if(forward == 3){
                shooting = true;
                nextPath = third_pick;
                shoot();
                if(steps == 2) {
                    shooting = false;
                    follower.followPath(gate);
                    forward = 4;
                    steps = 0;
                }
            }else if(forward == 4){
                follower.followPath(third_pick);
                forward = 4.5;
            }
            else if(forward == 4.5){
                follower.followPath(fourth_shoot);
                second_dump = false;
                forward = 5;
            }
            else if(forward == 5){
                shooting = true;
                nextPath = pick_after_stuff;
                shoot();
                if(steps == 2) {
                    shooting = false;
                    forward = 6;
                    steps = 0;
                }
            }else if(forward == 6){
                follower.followPath(fifth_shoot);
                forward = 7;
            }else if(forward == 7){
                if(!shooting){
                    pick_after_stuff = new Path(new BezierCurve(fourth_shoot.getLastControlPoint(), new Pose(2, -42)));
                    pick_after_stuff.setConstantHeadingInterpolation(Math.toRadians(-155));
                    fifth_shoot = new Path(new BezierLine(third_pick.getLastControlPoint(), fourth_shoot.getLastControlPoint()));
                    fifth_shoot.setConstantHeadingInterpolation(Math.toRadians(-155));

                }
                shooting = true;
                nextPath = pick_after_stuff;
                shoot();
                if(steps == 2) {
                    shooting = false;
                    forward = 6;
                    steps = 0;
                }
            }
        }
    }

    public void red_init(){
        first_shoot = new Path(new BezierLine(new Pose(123,-24), new Pose(100, -32)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(-180));
        first_pick = new Path(new BezierLine(first_shoot.getLastControlPoint(), new Pose(74, -32)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(-180));
        second_shoot = new Path(new BezierLine(first_pick.getLastControlPoint(), new Pose(86, -32)));
        second_shoot.setConstantHeadingInterpolation(Math.toRadians(-180));
        second_pick = new Path(new BezierLine(second_shoot.getLastControlPoint(), new Pose(54, -32)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(-180));
        third_shoot = new Path(new BezierLine(second_pick.getLastControlPoint(), second_shoot.getLastControlPoint()));
        third_shoot.setConstantHeadingInterpolation(Math.toRadians(-180));
        gate = new Path(new BezierCurve(third_shoot.getLastControlPoint(), new Pose(58, -32), new Pose(56, -39)));
        gate.setConstantHeadingInterpolation(Math.toRadians(-180));
        third_pick = new Path(new BezierLine(new Pose(43, -32), new Pose(33, -32)));
        third_pick.setConstantHeadingInterpolation(Math.toRadians(-180));
        fourth_shoot = new Path(new BezierLine(third_pick.getLastControlPoint(), new Pose(70, -9)));
        fourth_shoot.setConstantHeadingInterpolation(Math.toRadians(-180));
        pick_after_stuff = new Path(new BezierCurve(fourth_shoot.getLastControlPoint(), new Pose(58, -38), new Pose(56, -38), new Pose(2, -42)));
        pick_after_stuff.setConstantHeadingInterpolation(Math.toRadians(-180));
        fifth_shoot = new Path(new BezierLine(third_pick.getLastControlPoint(), fourth_shoot.getLastControlPoint()));
        fifth_shoot.setConstantHeadingInterpolation(Math.toRadians(-180));
        park = new Path(new BezierLine(fourth_shoot.getLastControlPoint(), new Pose(fourth_shoot.getLastControlPoint().getX(), fourth_shoot.getLastControlPoint().getY() + .1)));
        park.setConstantHeadingInterpolation(Math.toRadians(-155));
    }
    public void blue_init(){
        first_shoot = new Path(new BezierLine(new Pose(123,-24), new Pose(100, -44)));
        first_shoot.setConstantHeadingInterpolation(-180);
    }
    public void shoot(){
        if(shooting){
            if(steps == 0){
                shooting_time.reset();
                intake_system.flippy_pos = flippy_up;
                intake_system.blocking = false;
                steps = 1;
            }else if(steps == 1 && shooting_time.milliseconds() > 800){
                follower.followPath(nextPath);
                intake_system.blocking = true;
                intake_system.flippy_pos = flippy_hold;
                steps = 2;
            }
        }
    }
    public static double error = 2;
    public boolean close(Pose robot, Path path){
        boolean x_correct = abs(path.getLastControlPoint().getX() - robot.getX()) < error;
        boolean y_correct = abs(path.getLastControlPoint().getY() - robot.getY()) < error;
        return x_correct && y_correct;
    }



}
