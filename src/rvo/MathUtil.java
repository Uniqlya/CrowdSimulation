package rvo;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

class MathUtil {
    static final double EPSILON = 0.00001;

    static double det(Vector2D vector1, Vector2D vector2) {
        return vector1.getX() * vector2.getY() - vector1.getY() * vector2.getX();
    }

    static double leftOf(Vector2D point1, Vector2D point2, Vector2D point3) {
        return det(point1.subtract(point3), point2.subtract(point1));
    }
}
