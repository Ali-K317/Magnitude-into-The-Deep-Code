package org.firstinspires.ftc.teamcode.Into_The_Deep.Old.newAuto.teaching.teach.complexAuto;

public interface baseAutoInterface {
    void initialize();
    void setDirection();
    void setMode();
    void base(int targetPosFL, int targetPosFR, int targetPosBL, int targetPosBR, double speedFL, double speedFR, double speedBL, double speedBR);
    void movement(baseAutoEnum state, double inches,double rotation, double speed);
}
