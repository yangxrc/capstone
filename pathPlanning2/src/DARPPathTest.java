import java.awt.*;
import java.util.ArrayList;
import java.util.Vector;

public class DARPPathTest{

    public static void main(String[] args) {
        // ------------------------------
        //           DARP parameters
        // ------------------------------
        int droneNo = 2;  // Number of drones
        boolean notEqualPortions = false; // Equal portions
        double[] Rportions = new double[]{};  // Will be set to equal portions if not specified

        // Grid size (smaller grid)
        int rows = 7;
        int cols = 7;

        // Initialize a small grid (0 = free space)
        int[][] DARPgrid = new int[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                DARPgrid[i][j] = 0;
            }
        }

        // Initial positions of the drones
        DARPgrid[0][0] = 2; // Drone 1 starting position
        DARPgrid[2][2] = 2; // Drone 2 starting position

        // Ensure grid connectivity (no obstacles in this simple case)
        ConnectComponent G2G = new ConnectComponent();
        int[][] connectivityTest = new int[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                connectivityTest[i][j] = Math.abs(DARPgrid[i][j] - 1);
            }
        }
        G2G.compactLabeling(connectivityTest, new Dimension(cols, rows), true);
        if (G2G.getMaxLabel() > 1) {
            System.out.println("Grid has unreachable or closed shape regions.");
            return;
        }

        // Parameters for running DARP
        int MaxIter = 10000;
        double CCvariation = 0.01;
        double randomLevel = 0.0001;
        int dcells = 2;
        boolean importance = false;

        // Set equal portions if not specified
        if (!notEqualPortions) {
            Rportions = new double[droneNo];
            for (int i = 0; i < droneNo; i++) {
                Rportions[i] = 1.0 / droneNo;
            }
        }

        // Run DARP
        DARP problem = new DARP(rows, cols, DARPgrid, MaxIter, CCvariation, randomLevel, dcells, importance, Rportions);
        problem.constructAssignmentM();
        if (!problem.getSuccess()) {
            System.out.println("DARP did not find a valid solution.");
            return;
        }

        // Get assignment matrix and robot regions
        int[][] DARPAssignmentMatrix = problem.getAssignmentMatrix();
        ArrayList<boolean[][]> robotRegions = problem.getBinrayRobotRegions();
        ArrayList<Integer[]> initialPositions = problem.getRobotsInit();

        // Display assignment matrix
        System.out.println("DARP found a solution!");
        System.out.println("Assignment matrix:");
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                System.out.print(DARPAssignmentMatrix[i][j] + " ");
            }
            System.out.println();
        }

        // Calculate and display paths using CalculateTrajectories
        System.out.println("\nPaths for each drone:");
        for (int r = 0; r < droneNo; r++) {
            CalculateTrajectories trajectoryCalculator = new CalculateTrajectories(rows, cols, new Vector());
            trajectoryCalculator.initializeGraph(CalcRealBinaryReg(robotRegions.get(r), rows, cols), true);
            trajectoryCalculator.CalculatePathsSequence(4 * initialPositions.get(r)[0] * cols + 2 * initialPositions.get(r)[1]);
            ArrayList<Integer[]> pathSequence = trajectoryCalculator.getPathSequence();

            System.out.println("Drone " + (r + 1) + " path:");
            for (Integer[] step : pathSequence) {
                System.out.println("(" + step[0] + ", " + step[1] + ") -> (" + step[2] + ", " + step[3] + ")");
            }
        }
    }

    // Helper method to convert binary regions for path calculation
    private static boolean[][] CalcRealBinaryReg(boolean[][] BinrayRobotRegion, int rows, int cols) {
        boolean[][] RealBinrayRobotRegion = new boolean[2 * rows][2 * cols];
        for (int i = 0; i < 2 * rows; i++) {
            for (int j = 0; j < 2 * cols; j++) {
                RealBinrayRobotRegion[i][j] = BinrayRobotRegion[i / 2][j / 2];
            }
        }
        return RealBinrayRobotRegion;
    }
}
