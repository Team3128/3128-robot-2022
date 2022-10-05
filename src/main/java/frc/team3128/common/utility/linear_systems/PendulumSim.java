package frc.team3128.common.utility.linear_systems;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N0;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.team3128.Constants;

/**
 * Physics Simulation class to simulate a single pendulum
 * 
 * @author Sohan :o
 */
public class PendulumSim extends LinearSystemSim<N4,N1,N4> {

    private static final double G = 9.8; //Gravitational Constant
    private static Matrix<N2,N0> b = new MatBuilder<N2,N0>(Nat.N2(), Nat.N0()).fill();
    private static Matrix <N2,N2> c = Matrix.eye(Nat.N2());
    private static Matrix<N2,N0> d = new MatBuilder<N2,N0>(Nat.N2(), Nat.N0()).fill();

    /**
     * 
     * @param initialAngle The initial angle that the robot forms with the downward vertical in degrees (counter-clockwise)
     * @param armLength Length of the arm (in meters. I like SI Units)
     */
    public PendulumSim(double initialAngle, double armLength) {
        super(constructSystem(initialAngle, armLength));
        this.setState(new MatBuilder<N4,N1>(Nat.N4(), Nat.N1()).fill(initialAngle, armLength, 0, 0));
    }

    private static LinearSystem<N4,N1,N4> constructSystem(double initialAngle, double armLength) {
        initialAngle = 0;
        Matrix<N4,N4> aN = new MatBuilder<N4,N4>(Nat.N4(), Nat.N4())
        .fill(0,0,1,0,
        0,0,0,1,
        (-1*Constants.SimConstants.GRAVITATIONAL_CONSTANT*Math.cos(initialAngle)/armLength),
        (Constants.SimConstants.GRAVITATIONAL_CONSTANT*Math.sin(initialAngle)/(armLength*armLength)),0,0,
        0,0,0,0
        );

        Matrix<N4,N1> bN = new MatBuilder<N4,N1>(Nat.N4(), Nat.N1()).fill(0,0,0,1);
        Matrix<N4,N4> cN = Matrix.eye(Nat.N4());
        Matrix<N4,N1> dN = new MatBuilder<N4,N1>(Nat.N4(), Nat.N1()).fill(0,0,0,0);
        return new LinearSystem<>(aN,bN,cN,dN);
    }

}