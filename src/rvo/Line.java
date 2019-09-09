package rvo;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class Line {
    Vector2D direction = Vector2D.ZERO;
    Vector2D point = Vector2D.ZERO;

    public Line() { }

    public Line(Vector2D point, Vector2D direction) {
        this.direction = direction;
        this.point = point;
    }
}
