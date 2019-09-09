package app;

import rvo.Agent;
import utilPac.Util;

import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.util.List;

public class GUI extends JFrame implements KeyListener {
    private RDScene rd;
    private int height, width;
    private int scale = 25, stepF = 1;
    private int rightOff = 386, downOff = 424;
    private double logDiff;
    List<Integer> direction;

    GUI(int w, int h, RDScene rd) {
        width = w;
        height = h;
        this.rd = rd;
        direction = Util.intVecFromFile(RDScene.dataDir +"direction.txt");

        setSize(w, h);
        addKeyListener(this);
        setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        setVisible(true);
    }

    @Override
    public void keyPressed(KeyEvent e) {
        switch (e.getKeyChar()) {
            case 'w':
                downOff -= 2;
                break;
            case 's':
                downOff += 2;
                break;
            case 'a':
                rightOff -= 2;
                break;
            case 'd':
                rightOff += 2;
                break;
            case 'q':
                scale += 1;
                break;
            case  'e':
                if (scale > 1)
                    scale -= 1;
                break;
            case  'r':
                stepF++;
                break;
            case 'x':
                for(int i = 0; i < stepF; i++)
                    rd.step();
                break;
        }

        repaint();
    }

    @Override
    public void keyReleased(KeyEvent e) {
//        System.out.println("Rel");
    }

    @Override
    public void keyTyped(KeyEvent e) {
//        System.out.println("Type");
    }

    @Override
    public void paint(Graphics g) {
        super.paint(g);
        Graphics2D g2d = (Graphics2D)g;

        List<Agent> agents = rd.allAgents;

        g2d.drawString("r " + rightOff + " d " + downOff + " Scale " + scale, width-150, 100);

        AffineTransform af = new AffineTransform();
        af.translate(rightOff, downOff);
        af.scale(scale, -scale);
        g2d.transform(af);
        g2d.setStroke(new BasicStroke(2.0f/scale));

        for (int i = 0; i < agents.size(); i++) {
            Agent a = agents.get(i);
            double r = a.getRadius();
            double d = r * 2;
            Ellipse2D e = new Ellipse2D.Double(a.getPosition().getX() - r, a.getPosition().getY() - r, d, d);
            g2d.setColor(Color.RED);
            g2d.fill(e);
        }
        g.setColor(Color.BLACK);
        g2d.drawLine(-10, 0, 10, 0);
    }

    public static void main(String[] args) {
        RDScene b = new RDScene(300, "DML2/", "", 4);
        b.setupScene();
        GUI a = new GUI(800, 600, b);
    }
}
