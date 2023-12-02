package org.example;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;

public class Main2 extends JFrame {

    Panel2 panel = new Panel2();
    public static void main(String[] args) {
        EventQueue.invokeLater(new Runnable() {
            public void run() {
                new Main2().setVisible(true);
            }
        });
    }

    public Main2() {
        initComponents();
    }

    public void initComponents() {
        this.setPreferredSize(new Dimension(panel.width,panel.height));
        this.add(panel);
        this.pack();
        this.setVisible(true);
    }
}

class Panel2 extends JPanel {
    int x = 0;
    int y = 0;
    int width = 1800;
    int height = 1300;
    int offset = width/2;


    SlideModel slideModel = new SlideModel();

    public Panel2(){
        this.setVisible(true);
        this.addMouseMotionListener(new MouseAdapter() {
            public void mouseMoved(MouseEvent e) {
                x = e.getX();
                y = e.getY();
                repaint();
            }
        });
    }
    double[] oldPos;
    @Override
    public void paintComponent(Graphics g){
        super.paintComponent(g);
        g.setColor(Color.red);

        int adjX = x-this.getX()-offset;
        int adjY = (this.getHeight()/2)-y;
        //draw a line at a 60 degree angle from horizontal
        g.drawLine(convertX(0), convertY(0), convertX(400*Math.cos(Math.toRadians(60))), convertY(400*Math.sin(Math.toRadians(60))));

        double[] pos = slideModel.inverseKinematics(adjX, adjY);
        if(pos != null){
            drawArm(g, pos[0], pos[1]);
            oldPos = pos;
        }

        else if(oldPos != null)
            drawArm(g, oldPos[0], oldPos[1]);

    }

    public int convertX(double x){
        return (int) (x+offset);
    }
    public int convertY(double y){
        return (int) (this.getHeight()/2-y);
    }

    //function to draw the arm based on the slide position which is on a 60 degree angle from the ground and the arm angle
    public void drawArm(Graphics g, double slidePosition, double armAngle){
        //calculate the x and y position of the arm pivot
        armAngle -= Math.toRadians(180);
        double armPX = (Math.cos(slideModel.SLIDEANGLE)*slidePosition) - (Math.cos(armAngle)*slideModel.ARMLENGTH);
        double armPY = (Math.sin(slideModel.SLIDEANGLE)*slidePosition) - (Math.sin(armAngle)*slideModel.ARMLENGTH);
        //draw the arm
        g.drawLine(convertX(armPX), convertY(armPY), convertX(armPX+(Math.cos(armAngle)*slideModel.ARMLENGTH)), convertY(armPY+(Math.sin(armAngle)*slideModel.ARMLENGTH)));
        //draw the wrist which is at a constant 60 degree angle relative to the ground
        g.drawLine(convertX(armPX), convertY(armPY), convertX(armPX+(Math.cos(slideModel.WRISTANGLE)*slideModel.WRISTLENGTH)), convertY(armPY+(Math.sin(slideModel.WRISTANGLE)*slideModel.WRISTLENGTH)));
    }

}