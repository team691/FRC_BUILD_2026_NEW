package frc.robot.utils;
// 2.6x^{2}-19.5x+55
public class PolynomialRegressor {
    double[] coeffs = {2.6, -19.5, 55.0};
    public double speed(double X) {
        double res = Math.pow(X, 2)*coeffs[0] + Math.pow(X, 1)*coeffs[1] + coeffs[2];
        return res;
    }
}
