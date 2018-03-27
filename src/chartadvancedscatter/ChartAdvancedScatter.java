package chartadvancedscatter;

import javafx.application.Application;
import javafx.geometry.Side;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.ScatterChart;
import javafx.scene.chart.XYChart;
import javafx.stage.Stage;
import chartadvancedscatter.Vector3;
import chartadvancedscatter.Quaternion;


public class ChartAdvancedScatter extends Application {
    // Chart Axis boundaries
    final static double chartScale = 5d;
    final double chartHorMin = -1.45d*chartScale;
    final double chartHorMax =  1.45d*chartScale;
    final double chartVirMin = -1d*chartScale;
    final double chartVirMax =  1d*chartScale;

    // Photo Diode Info
    final static double maxAoI = 90d;
    final static double relPowerRatioVsAoI = 1d/maxAoI;  // Approx to linear 0.9==25deg
    final static double maxDist = 20d;
    final static double relPowerRatioVsDist = 1d/maxDist; // Approx drops to zero at 20m
    final static double minRelPower = 0.05;
    
    // Sensor Cluster Info
    static int CLUSTER_SIZE = 32;
    Vector3 clusterOriginPosition = new Vector3(0d, 0, 0); // meters
    Vector3[] clusterSensorPositions = new Vector3[CLUSTER_SIZE]; // Meters
    Vector3[] clusterSensorNormalPositions = new Vector3[CLUSTER_SIZE]; // Meters
    Spherical3 clusterToBase1Spherical = new Spherical3();
    
    // Base1 info
    Vector3 base1OriginPosition = new Vector3(2.0d,0,0); // meters
    
    // Base to Sensor relative info
//    Spherical3 base1ToClusterSpherical = new Spherical3(0, 0, 0.5);
    Spherical3[] base1ToSensorSpherical = new Spherical3[CLUSTER_SIZE];
    Vector3[] base1ToSensorVector = new Vector3[CLUSTER_SIZE];
    double[] base1ToSensorAoI = new double[CLUSTER_SIZE];
    double[] base1ToSensorRelativePower = new double[CLUSTER_SIZE];
    boolean[] base1ToSensorVisible = new boolean[CLUSTER_SIZE];

    // General items
    Vector3 zeroVector = new Vector3(0, 0, 0); // meters

    // =================================================================
    // =================================================================
    //                    EXPOSED POSER VARIABLES
    // =================================================================
    // =================================================================
    final static double icoSphereRadius = 0.5d; // Use to determine bearing estimates
    boolean[] icoSpherePointWithinAoI = new boolean[ICOSPHERE_POINTS];
    final static double sensorMaxFoR = 75d; // Use to determine bearing estimates
    Vector3 vectorEstimateFromIcoSphere = new Vector3();
    Vector3 baseEstPosnFromIcoSphere = new Vector3();
    Spherical3 bearingEstimateFromIcoSphere = new Spherical3();

    
    
    // Get the estimated range from the initial bearing and 
    private void getInitialRangeFromSensorAngles() {
        System.out.printf("\n\r\n\rgetInitialRangeFromSensorAngles:");
        double maxMeasuredAngle = 0d;
        double tempEstimatedAngle = 0d;
        double elDiff;
        double azDiff;
        int heldOuter = 0;
        int heldInner = 1;
        Vector3 estimateOffset = new Vector3();

        // 
        for (int outerCount = 0; outerCount < (CLUSTER_SIZE-1); outerCount++) {
            for (int innerCount = (outerCount+1); innerCount < CLUSTER_SIZE; innerCount++) {
                azDiff = base1ToSensorSpherical[outerCount].az - base1ToSensorSpherical[innerCount].az;
                elDiff = base1ToSensorSpherical[outerCount].el - base1ToSensorSpherical[innerCount].el;
                tempEstimatedAngle = Math.sqrt((azDiff*azDiff)+(elDiff*elDiff));
                if (tempEstimatedAngle > maxMeasuredAngle){
                    maxMeasuredAngle = tempEstimatedAngle;
                    heldOuter = outerCount;
                    heldInner = innerCount;
                }
            }
        }
        System.out.printf("\n\r   Base Psn: (%2.4f, %2.4f, %2.4f)", base1OriginPosition.x, base1OriginPosition.y, base1OriginPosition.z);
        System.out.printf("\n\r   Clus Psn: (%2.4f, %2.4f, %2.4f)", clusterOriginPosition.x, clusterOriginPosition.y, clusterOriginPosition.z);
        System.out.printf("\n\r   Esti Psn: (%2.4f, %2.4f, %2.4f)", vectorEstimateFromIcoSphere.x, vectorEstimateFromIcoSphere.y, vectorEstimateFromIcoSphere.z);
        estimateOffset.set(vectorEstimateFromIcoSphere);
        estimateOffset.sub(clusterOriginPosition);
        System.out.printf("\n\r   First Sensor(%d), Second Sensor(%d) - measured angle (%2.4f)", heldOuter, heldInner, Math.toDegrees(maxMeasuredAngle));

        // Get calculated and the angle from measured data
        tempEstimatedAngle = angleThreePoints(clusterSensorPositions[heldOuter], vectorEstimateFromIcoSphere, clusterSensorPositions[heldInner]);
        System.out.printf("\n\r   Angle to estimate (%2.4f)", Math.toDegrees(tempEstimatedAngle));
        System.out.printf("\n\r   Angle to actual (%2.4f)", Math.toDegrees(maxMeasuredAngle));

        estimateOffset.scale (Math.toDegrees(tempEstimatedAngle)/Math.toDegrees(maxMeasuredAngle));
        vectorEstimateFromIcoSphere.set(estimateOffset);
        vectorEstimateFromIcoSphere.add(clusterOriginPosition);
        tempEstimatedAngle = angleThreePoints(clusterSensorPositions[heldOuter], vectorEstimateFromIcoSphere, clusterSensorPositions[heldInner]);
        System.out.printf("\n\r   EST Psn: (%2.4f, %2.4f, %2.4f)", vectorEstimateFromIcoSphere.x, vectorEstimateFromIcoSphere.y, vectorEstimateFromIcoSphere.z);
        System.out.printf("\n\r   Angle to estimate (%2.4f)", Math.toDegrees(tempEstimatedAngle));
    }
    
    
    // Run through the points of an IcoSphere and see which are covered by the
    // FoR of each of the sensors which have been lit by the (H/V) laser sweeps.
    // Take an average of the bearings for those IcoSphere points to get an
    // initial bearing for the poser.
    private void getInitialBearingFromIcoSphere() {
        System.out.printf("\n\rgetInitialBearingFromIcoSphere:");
        // IcoSphere items
        int tempCount = 0;
        double tempIcoSphereAoI = 0.0d;
        Vector3 tempIcoSphereVector = new Vector3();
        Vector3 tempNormalsVector = new Vector3();
        Spherical3 tempIcoSphereSpherical = new Spherical3();
        
        vectorEstimateFromIcoSphere.set(0, 0, 0);

        for (int spCount = 0; spCount < ICOSPHERE_POINTS; spCount++) {
            icoSpherePointWithinAoI[spCount] = true;
        }

        // Search for positions in the IcoSphere to give a general 
        for (int count = 0; count < CLUSTER_SIZE; count++) {
            for (int spCount = 0; spCount < ICOSPHERE_POINTS; spCount++) {
                if ((icoSpherePointWithinAoI[spCount])&&(base1ToSensorVisible[count])) {
                    tempIcoSphereVector.set(icoSphereCoords[spCount][0]*icoSphereRadius, icoSphereCoords[spCount][1]*icoSphereRadius, icoSphereCoords[spCount][2]*icoSphereRadius);
                    tempIcoSphereAoI = angleThreePoints(clusterSensorNormalPositions[count], clusterSensorPositions[count], tempIcoSphereVector);
                    icoSpherePointWithinAoI[spCount] = (tempIcoSphereAoI < Math.toRadians(sensorMaxFoR));
                }
            }
        }

        System.out.printf("\n\r   IcoSphere Lit: ");
        tempIcoSphereVector.set(0d, 0d, 0d);
        for (int spCount = 0; spCount < ICOSPHERE_POINTS; spCount++) {
            if (icoSpherePointWithinAoI[spCount]) {
                tempIcoSphereVector.set(icoSphereCoords[spCount][0], icoSphereCoords[spCount][1], icoSphereCoords[spCount][2]);
                vectorEstimateFromIcoSphere.add(tempIcoSphereVector);
                tempCount++;
                System.out.printf("   %d", spCount);
            }
        }
        
        System.out.printf("\n\r   AVG Vector(%d): (%2.8fm, %2.8fm, %2.8fm)", tempCount, vectorEstimateFromIcoSphere.x, vectorEstimateFromIcoSphere.y, vectorEstimateFromIcoSphere.z);
        vectorEstimateFromIcoSphere.x /= tempCount;
        vectorEstimateFromIcoSphere.y /= tempCount;
        vectorEstimateFromIcoSphere.z /= tempCount;
        System.out.printf("\n\r   AVG Vector(1): (%2.8fm, %2.8fm, %2.8fm)", vectorEstimateFromIcoSphere.x, vectorEstimateFromIcoSphere.y, vectorEstimateFromIcoSphere.z);
        baseEstPosnFromIcoSphere.set(clusterOriginPosition);
        baseEstPosnFromIcoSphere.sub(vectorEstimateFromIcoSphere);
        System.out.printf("\n\r   Base Est: (%2.8fm, %2.8fm, %2.8fm)", baseEstPosnFromIcoSphere.x, baseEstPosnFromIcoSphere.y, baseEstPosnFromIcoSphere.z);
    }


    // Create the values from the system, giving which sensors within the cluster
    // are lit and what the bearing is to that sensor from the base
    // ALL ANGLES ARE BASED AROUND THE CLUSTER BEING AT ZERO
    private void getLitSensorAngles() {
        System.out.printf("\n\r\n\rgetLitSensorAngles:");

        Vector3 clusterOriginToBase = new Vector3(base1OriginPosition).add(clusterOriginPosition);
        clusterToBase1Spherical.setFromVector3(clusterOriginToBase);
        int count = 0;
        

        // Setup sensor cluster detector positions and normal positions (wrt to the detector)
        for (count = 0; count < CLUSTER_SIZE; count++) {
            clusterSensorPositions[count] = new Vector3();
            clusterSensorPositions[count].x = devicePoints[count][2];
            clusterSensorPositions[count].y = devicePoints[count][0];
            clusterSensorPositions[count].z = devicePoints[count][1];

            clusterSensorNormalPositions[count] = new Vector3();
            clusterSensorNormalPositions[count].x = clusterSensorPositions[count].x + deviceNormals[count][2]; // Meters
            clusterSensorNormalPositions[count].y = clusterSensorPositions[count].y + deviceNormals[count][0]; // Meters
            clusterSensorNormalPositions[count].z = clusterSensorPositions[count].z + deviceNormals[count][1]; // Meters
            
            // Transform for orientation, rotates 180 deg in az
//            clusterSensorPositions[count].x *= -1d;
//            clusterSensorPositions[count].y *= -1d;
//            clusterSensorNormalPositions[count].x *= -1d;
//            clusterSensorNormalPositions[count].y *= -1d;
            
            // Get the Spherical Co-ords for the sensors
            base1ToSensorVector[count] = new Vector3(base1OriginPosition);
            base1ToSensorVector[count].add(clusterSensorPositions[count]);
            base1ToSensorSpherical[count] = new Spherical3();
            base1ToSensorSpherical[count].setFromVector3(base1ToSensorVector[count]);
            
            // work out the angle to the base from each sensor normal
            base1ToSensorAoI[count] = angleThreePoints(base1OriginPosition, clusterSensorPositions[count], clusterSensorNormalPositions[count]);
            base1ToSensorVisible[count] = (Math.toDegrees(base1ToSensorAoI[count]) < 80d);
        }

        // Run through the cluster and print the results
        System.out.printf("\n\r   Base 1 Posn (x:%2.4f, y:%2.4f, z:%2.4f)", base1OriginPosition.x, base1OriginPosition.y, base1OriginPosition.z);
        System.out.printf("\n\r   Cluster 1 Posn (x:%2.4f, y:%2.4f, z:%2.4f)", clusterOriginPosition.x, clusterOriginPosition.y, clusterOriginPosition.z);
        System.out.printf("\n\r   Clus to Base Brng (a:%3.4f, e:%3.4f, r:%3.4f), ", Math.toDegrees(clusterToBase1Spherical.az), Math.toDegrees(clusterToBase1Spherical.el), clusterToBase1Spherical.r);
        System.out.print("\n\r\n\r   Sens: (Base1 centric x, y, z), (Az Angle, El Angle, Range), AoI, Relative Power");
        for (count = 0; count < CLUSTER_SIZE; count++) {
            if(base1ToSensorVisible[count]){
                System.out.printf("\n\r   %d: ", count);
                System.out.printf("(%2.4f, %2.4f, %2.4f), ", base1ToSensorVector[count].x, base1ToSensorVector[count].y, base1ToSensorVector[count].z);
                System.out.printf("(%3.4f, %3.4f, %3.4f), ", Math.toDegrees(base1ToSensorSpherical[count].az), Math.toDegrees(base1ToSensorSpherical[count].el), base1ToSensorSpherical[count].r);
                System.out.printf("%.3f, %.3f, ", Math.toDegrees(base1ToSensorAoI[count]), base1ToSensorRelativePower[count]);
            }
        }
        System.out.print("\n\r");
    }

    
    
    
    



    // Create the scatter chart
    private void initChart(Stage primaryStage) {
        Group root = new Group();
        primaryStage.setScene(new Scene(root));
        root.getChildren().add(createChart01());
    }

    protected ScatterChart<Number, Number> createChart01() {
        final NumberAxis xAxis = new NumberAxis("Az",chartHorMin,chartHorMax,1);
        xAxis.setSide(Side.TOP);
        final NumberAxis yAxis = new NumberAxis("El",chartVirMin,chartVirMax,1);
        yAxis.setSide(Side.RIGHT);
        final ScatterChart<Number,Number> sc = new ScatterChart<Number,Number>(xAxis,yAxis);
        
        if (base1ToSensorVisible[ 0]) { XYChart.Series<Number, Number> seriesSensors00 = new XYChart.Series<Number, Number>(); seriesSensors00.setName("0"); seriesSensors00.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 0].az), Math.toDegrees(base1ToSensorSpherical[ 0].el)));sc.getData().add(seriesSensors00);}
        if (base1ToSensorVisible[ 1]) { XYChart.Series<Number, Number> seriesSensors01 = new XYChart.Series<Number, Number>(); seriesSensors01.setName("1"); seriesSensors01.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 1].az), Math.toDegrees(base1ToSensorSpherical[ 1].el)));sc.getData().add(seriesSensors01);}
        if (base1ToSensorVisible[ 2]) { XYChart.Series<Number, Number> seriesSensors02 = new XYChart.Series<Number, Number>(); seriesSensors02.setName("2"); seriesSensors02.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 2].az), Math.toDegrees(base1ToSensorSpherical[ 2].el)));sc.getData().add(seriesSensors02);}
        if (base1ToSensorVisible[ 3]) { XYChart.Series<Number, Number> seriesSensors03 = new XYChart.Series<Number, Number>(); seriesSensors03.setName("3"); seriesSensors03.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 3].az), Math.toDegrees(base1ToSensorSpherical[ 3].el)));sc.getData().add(seriesSensors03);}
        if (base1ToSensorVisible[ 4]) { XYChart.Series<Number, Number> seriesSensors04 = new XYChart.Series<Number, Number>(); seriesSensors04.setName("4"); seriesSensors04.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 4].az), Math.toDegrees(base1ToSensorSpherical[ 4].el)));sc.getData().add(seriesSensors04);}
        if (base1ToSensorVisible[ 5]) { XYChart.Series<Number, Number> seriesSensors05 = new XYChart.Series<Number, Number>(); seriesSensors05.setName("5"); seriesSensors05.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 5].az), Math.toDegrees(base1ToSensorSpherical[ 5].el)));sc.getData().add(seriesSensors05);}
        if (base1ToSensorVisible[ 6]) { XYChart.Series<Number, Number> seriesSensors06 = new XYChart.Series<Number, Number>(); seriesSensors06.setName("6"); seriesSensors06.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 6].az), Math.toDegrees(base1ToSensorSpherical[ 6].el)));sc.getData().add(seriesSensors06);}
        if (base1ToSensorVisible[ 7]) { XYChart.Series<Number, Number> seriesSensors07 = new XYChart.Series<Number, Number>(); seriesSensors07.setName("7"); seriesSensors07.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 7].az), Math.toDegrees(base1ToSensorSpherical[ 7].el)));sc.getData().add(seriesSensors07);}
        if (base1ToSensorVisible[ 8]) { XYChart.Series<Number, Number> seriesSensors08 = new XYChart.Series<Number, Number>(); seriesSensors08.setName("8"); seriesSensors08.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 8].az), Math.toDegrees(base1ToSensorSpherical[ 8].el)));sc.getData().add(seriesSensors08);}
        if (base1ToSensorVisible[ 9]) { XYChart.Series<Number, Number> seriesSensors09 = new XYChart.Series<Number, Number>(); seriesSensors09.setName("9"); seriesSensors09.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 9].az), Math.toDegrees(base1ToSensorSpherical[ 9].el)));sc.getData().add(seriesSensors09);}
        if (base1ToSensorVisible[10]) { XYChart.Series<Number, Number> seriesSensors10 = new XYChart.Series<Number, Number>(); seriesSensors10.setName("10"); seriesSensors10.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[10].az), Math.toDegrees(base1ToSensorSpherical[10].el)));sc.getData().add(seriesSensors10);}
        if (base1ToSensorVisible[11]) { XYChart.Series<Number, Number> seriesSensors11 = new XYChart.Series<Number, Number>(); seriesSensors11.setName("11"); seriesSensors11.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[11].az), Math.toDegrees(base1ToSensorSpherical[11].el)));sc.getData().add(seriesSensors11);}
        if (base1ToSensorVisible[12]) { XYChart.Series<Number, Number> seriesSensors12 = new XYChart.Series<Number, Number>(); seriesSensors12.setName("12"); seriesSensors12.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[12].az), Math.toDegrees(base1ToSensorSpherical[12].el)));sc.getData().add(seriesSensors12);}
        if (base1ToSensorVisible[13]) { XYChart.Series<Number, Number> seriesSensors13 = new XYChart.Series<Number, Number>(); seriesSensors13.setName("13"); seriesSensors13.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[13].az), Math.toDegrees(base1ToSensorSpherical[13].el)));sc.getData().add(seriesSensors13);}
        if (base1ToSensorVisible[14]) { XYChart.Series<Number, Number> seriesSensors14 = new XYChart.Series<Number, Number>(); seriesSensors14.setName("14"); seriesSensors14.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[14].az), Math.toDegrees(base1ToSensorSpherical[14].el)));sc.getData().add(seriesSensors14);}
        if (base1ToSensorVisible[15]) { XYChart.Series<Number, Number> seriesSensors15 = new XYChart.Series<Number, Number>(); seriesSensors15.setName("15"); seriesSensors15.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[15].az), Math.toDegrees(base1ToSensorSpherical[15].el)));sc.getData().add(seriesSensors15);}
        if (base1ToSensorVisible[16]) { XYChart.Series<Number, Number> seriesSensors16 = new XYChart.Series<Number, Number>(); seriesSensors16.setName("16"); seriesSensors16.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[16].az), Math.toDegrees(base1ToSensorSpherical[16].el)));sc.getData().add(seriesSensors16);}
        if (base1ToSensorVisible[17]) { XYChart.Series<Number, Number> seriesSensors17 = new XYChart.Series<Number, Number>(); seriesSensors17.setName("17"); seriesSensors17.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[17].az), Math.toDegrees(base1ToSensorSpherical[17].el)));sc.getData().add(seriesSensors17);}
        if (base1ToSensorVisible[18]) { XYChart.Series<Number, Number> seriesSensors18 = new XYChart.Series<Number, Number>(); seriesSensors18.setName("18"); seriesSensors18.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[18].az), Math.toDegrees(base1ToSensorSpherical[18].el)));sc.getData().add(seriesSensors18);}
        if (base1ToSensorVisible[19]) { XYChart.Series<Number, Number> seriesSensors19 = new XYChart.Series<Number, Number>(); seriesSensors19.setName("19"); seriesSensors19.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[19].az), Math.toDegrees(base1ToSensorSpherical[19].el)));sc.getData().add(seriesSensors19);}
        if (base1ToSensorVisible[20]) { XYChart.Series<Number, Number> seriesSensors20 = new XYChart.Series<Number, Number>(); seriesSensors20.setName("20"); seriesSensors20.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[20].az), Math.toDegrees(base1ToSensorSpherical[20].el)));sc.getData().add(seriesSensors20);}
        if (base1ToSensorVisible[21]) { XYChart.Series<Number, Number> seriesSensors21 = new XYChart.Series<Number, Number>(); seriesSensors21.setName("21"); seriesSensors21.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[21].az), Math.toDegrees(base1ToSensorSpherical[21].el)));sc.getData().add(seriesSensors21);}
        if (base1ToSensorVisible[22]) { XYChart.Series<Number, Number> seriesSensors22 = new XYChart.Series<Number, Number>(); seriesSensors22.setName("22"); seriesSensors22.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[22].az), Math.toDegrees(base1ToSensorSpherical[22].el)));sc.getData().add(seriesSensors22);}
        if (base1ToSensorVisible[23]) { XYChart.Series<Number, Number> seriesSensors23 = new XYChart.Series<Number, Number>(); seriesSensors23.setName("23"); seriesSensors23.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[23].az), Math.toDegrees(base1ToSensorSpherical[23].el)));sc.getData().add(seriesSensors23);}
        if (base1ToSensorVisible[24]) { XYChart.Series<Number, Number> seriesSensors24 = new XYChart.Series<Number, Number>(); seriesSensors24.setName("24"); seriesSensors24.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[24].az), Math.toDegrees(base1ToSensorSpherical[24].el)));sc.getData().add(seriesSensors24);}
        if (base1ToSensorVisible[25]) { XYChart.Series<Number, Number> seriesSensors25 = new XYChart.Series<Number, Number>(); seriesSensors25.setName("25"); seriesSensors25.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[25].az), Math.toDegrees(base1ToSensorSpherical[25].el)));sc.getData().add(seriesSensors25);}
        if (base1ToSensorVisible[26]) { XYChart.Series<Number, Number> seriesSensors26 = new XYChart.Series<Number, Number>(); seriesSensors26.setName("26"); seriesSensors26.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[26].az), Math.toDegrees(base1ToSensorSpherical[26].el)));sc.getData().add(seriesSensors26);}
        if (base1ToSensorVisible[27]) { XYChart.Series<Number, Number> seriesSensors27 = new XYChart.Series<Number, Number>(); seriesSensors27.setName("27"); seriesSensors27.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[27].az), Math.toDegrees(base1ToSensorSpherical[27].el)));sc.getData().add(seriesSensors27);}
        if (base1ToSensorVisible[28]) { XYChart.Series<Number, Number> seriesSensors28 = new XYChart.Series<Number, Number>(); seriesSensors28.setName("28"); seriesSensors28.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[28].az), Math.toDegrees(base1ToSensorSpherical[28].el)));sc.getData().add(seriesSensors28);}
        if (base1ToSensorVisible[29]) { XYChart.Series<Number, Number> seriesSensors29 = new XYChart.Series<Number, Number>(); seriesSensors29.setName("29"); seriesSensors29.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[29].az), Math.toDegrees(base1ToSensorSpherical[29].el)));sc.getData().add(seriesSensors29);}
        if (base1ToSensorVisible[30]) { XYChart.Series<Number, Number> seriesSensors30 = new XYChart.Series<Number, Number>(); seriesSensors30.setName("30"); seriesSensors30.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[30].az), Math.toDegrees(base1ToSensorSpherical[30].el)));sc.getData().add(seriesSensors30);}
        if (base1ToSensorVisible[31]) { XYChart.Series<Number, Number> seriesSensors31 = new XYChart.Series<Number, Number>(); seriesSensors31.setName("31"); seriesSensors31.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[31].az), Math.toDegrees(base1ToSensorSpherical[31].el)));sc.getData().add(seriesSensors31);}

        return sc;
    }
    
    
    double angleThreePoints(Vector3 Start, Vector3 Centre, Vector3 End) {
        double V1x = Start.x-Centre.x; double V1y = Start.y-Centre.y; double V1z = Start.z-Centre.z;
        double V2x = End.x-Centre.x; double V2y = End.y-Centre.y; double V2z = End.z-Centre.z;
        double V1mag = Math.sqrt(V1x*V1x+V1y*V1y+V1z*V1z);
        double V2mag = Math.sqrt(V2x*V2x+V2y*V2y+V2z*V2z);
        double V1Normx = V1x/V1mag; double V1Normy = V1y/V1mag; double V1Normz = V1z/V1mag; 
        double V2Normx = V2x/V2mag; double V2Normy = V2y/V2mag; double V2Normz = V2z/V2mag; 
        double Vdot = V1Normx*V2Normx+V1Normy*V2Normy+V1Normz*V2Normz;
        double AngAB = Math.acos(Vdot);
        return AngAB;
    }

    
    
    @Override public void start(Stage primaryStage) throws Exception {
        getLitSensorAngles();
        getInitialBearingFromIcoSphere();
        getInitialRangeFromSensorAngles();
        
        initChart(primaryStage);
        primaryStage.show();
        System.out.printf("\n\r\n\rDONE.\n\r");
    }

    /**
     * The main() method is ignored in correctly deployed JavaFX 
     * application. main() serves only as fallback in case the 
     * application can not be launched through deployment artifacts,
     * e.g., in IDEs with limited FX support. NetBeans ignores main().
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        launch(args);
    }
    
    
        static double[][] devicePoints = {
        {0.08518143743276596,0.017062144353985786,0.04640356823801994},
        {0.09299874305725098,-9.77110757958144e-05,0.03490303456783295},
        {0.0866357758641243,0.016550032421946526,0.020586593076586723},
        {0.0896136462688446,0.029156366363167763,0.0296088345348835},
        {0.07996707409620285,0.04522520303726196,0.03478708118200302},
        {0.05082200840115547,0.0525379441678524,0.03328508138656616},
        {0.02431630529463291,0.0200039092451334,0.05943312123417854},
        {0.04736604541540146,0.03358921408653259,0.05357927456498146},
        {0.04778143763542175,-0.034000154584646225,0.05348391830921173},
        {0.05795735865831375,-3.651010774774477e-05,0.05651696398854256},
        {0.02757195383310318,-0.051707036793231964,0.046649035066366196},
        {0.05145823583006859,-0.05293474718928337,0.03312348574399948},
        {0.08054577559232712,-0.04522349312901497,0.03467874228954315},
        {0.08995519578456879,-0.029309064149856567,0.02968563325703144},
        {0.0868583470582962,-0.016645202413201332,0.020546138286590576},
        {0.08528480678796768,-0.01717553101480007,0.04645363613963127},
        {-0.04789695516228676,0.03364776074886322,0.05359702929854393},
        {-0.02451552450656891,0.020054345950484276,0.05939493328332901},
        {-0.05120636895298958,0.05282235145568848,0.033184923231601715},
        {-0.08026022464036942,0.045291341841220856,0.034813448786735535},
        {-0.08975434303283691,0.02939225733280182,0.029623806476593018},
        {-0.08676112443208694,0.01667257957160473,0.02072800137102604},
        {-0.09296955168247223,0.00019559808424673975,0.034909311681985855},
        {-0.08538919687271118,0.01735016517341137,0.046313270926475525},
        {-0.08526882529258728,-0.017100226134061813,0.046251364052295685},
        {-0.08669501543045044,-0.016456371173262596,0.020705312490463257},
        {-0.08958882093429565,-0.0292942076921463,0.029727233573794365},
        {-0.08019855618476868,-0.04522521793842316,0.0346868671476841},
        {-0.05091847851872444,-0.052784282714128494,0.03316209465265274},
        {-0.027258513495326042,-0.051615241914987564,0.04688679054379463},
        {-0.0580756776034832,6.801447852922138e-06,0.05650037154555321},
        {-0.047557104378938675,-0.03394269943237305,0.0535212866961956}
        };
    static double[][] deviceNormals = {
        {0.6565292477607727,0.08003702759742737,0.7500423192977905},
        {1,0,0},
        {0.9510334134101868,0.1922958791255951,-0.24198685586452484},
        {0.8633409738540649,0.26114100217819214,-0.43179601430892944},
        {0.5620832443237305,0.8270804286003113,-0.0007020003395155072},
        {0.5567418932914734,0.8186168074607849,-0.1410849690437317},
        {0.12751400470733643,0.36096900701522827,0.9238190054893494},
        {0.19732795655727386,0.7212077975273132,0.6640188097953796},
        {0.19732792675495148,-0.7205037474632263,0.6647827625274658},
        {0.4602000117301941,0.003066000062972307,0.8878099918365479},
        {0.025263000279664993,-0.7483329772949219,0.6628419756889343},
        {0.5567419528961182,-0.8187658786773682,-0.14021699130535126},
        {0.5620829463005066,-0.8270809054374695,0.00017499997920822352},
        {0.8633410930633545,-0.26159802079200745,-0.4315190613269806},
        {0.9510335326194763,-0.19255191087722778,-0.24178287386894226},
        {0.6565289497375488,-0.07924199104309082,0.7501268982887268},
        {-0.19732795655727386,0.7212077975273132,0.6640188097953796},
        {-0.12751400470733643,0.36096900701522827,0.9238190054893494},
        {-0.5567418932914734,0.8186168074607849,-0.1410849690437317},
        {-0.5620832443237305,0.8270804286003113,-0.0007020003395155072},
        {-0.8633409738540649,0.26114100217819214,-0.43179601430892944},
        {-0.9510334134101868,0.1922958791255951,-0.24198685586452484},
        {-1,0,0},
        {-0.6565292477607727,0.08003702759742737,0.7500423192977905},
        {-0.6565289497375488,-0.07924199104309082,0.7501268982887268},
        {-0.9510335326194763,-0.19255191087722778,-0.24178287386894226},
        {-0.8633410930633545,-0.26159802079200745,-0.4315190613269806},
        {-0.5620829463005066,-0.8270809054374695,0.00017499997920822352},
        {-0.5567419528961182,-0.8187658786773682,-0.14021699130535126},
        {-0.025263000279664993,-0.7483329772949219,0.6628419756889343},
        {-0.4602000117301941,0.003066000062972307,0.8878099918365479},
        {-0.19732792675495148,-0.7205037474632263,0.6647827625274658}
        };
    
    static int ICOSPHERE_POINTS = 320;
    static double[][] icoSphereCoords = {
    {0.048151, -0.14819, -0.987786},
    {0.722992, -0.412071, -0.554509},
    {-0.12606, -0.091587, -0.987786},
    {-0.12606, 0.091587, -0.987786},
    {0.048151, 0.14819, -0.987786},
    {0.819296, -0.412071, -0.398687},
    {-0.138726, -0.906533, -0.398689},
    {-0.905035, -0.148193, -0.398685},
    {-0.420608, 0.814945, -0.39869},
    {0.645086, 0.651853, -0.398688},
    {0.771146, -0.560264, -0.302388},
    {-0.294548, -0.906533, -0.302388},
    {-0.953185, 0, -0.302386},
    {-0.294548, 0.906533, -0.302389},
    {0.771146, 0.560265, -0.302387},
    {0.342697, -0.758339, 0.554509},
    {-0.615326, -0.560263, 0.554509},
    {-0.722992, 0.412072, 0.554509},
    {0.168486, 0.814943, 0.554509},
    {0.827125, 0.091587, 0.554505},
    {0.12606, 0.091587, 0.987786},
    {0.391194, 0.09145, 0.915753},
    {0.644124, 0.091451, 0.759434},
    {0.233376, 0.169558, 0.957489},
    {0.207859, 0.343786, 0.915754},
    {0.516845, 0.183046, 0.836281},
    {0.49112, 0.35682, 0.794658},
    {0.333798, 0.434985, 0.836282},
    {0.286017, 0.584336, 0.759438},
    {0.752031, 0.16956, 0.636945},
    {0.726114, 0.343786, 0.595458},
    {0.598705, 0.434986, 0.672561},
    {0.551342, 0.584336, 0.59546},
    {0.393647, 0.662827, 0.636948},
    {0.342697, 0.758339, 0.554509},
    {-0.048151, 0.148191, 0.987786},
    {0.033908, 0.400305, 0.915754},
    {0.112066, 0.640855, 0.759438},
    {-0.089143, 0.274349, 0.957489},
    {-0.262729, 0.30392, 0.915754},
    {-0.014375, 0.548111, 0.836282},
    {-0.187594, 0.577343, 0.794659},
    {-0.310547, 0.451877, 0.836282},
    {-0.467354, 0.452586, 0.759438},
    {0.071126, 0.767619, 0.636948},
    {-0.102585, 0.796808, 0.595461},
    {-0.228689, 0.703819, 0.672562},
    {-0.385367, 0.704925, 0.59546},
    {-0.508743, 0.579205, 0.636947},
    {-0.615326, 0.560263, 0.554509},
    {-0.155816, 0, 0.987786},
    {-0.370234, 0.155951, 0.915754},
    {-0.574859, 0.304617, 0.759438},
    {-0.28847, 0, 0.957489},
    {-0.370234, -0.155951, 0.915754},
    {-0.525729, 0.155706, 0.836281},
    {-0.607057, 0, 0.794658},
    {-0.525729, -0.155706, 0.836281},
    {-0.574859, -0.304617, 0.759438},
    {-0.708071, 0.304853, 0.636946},
    {-0.78951, 0.148666, 0.59546},
    {-0.740043, 0, 0.67256},
    {-0.78951, -0.148666, 0.59546},
    {-0.708071, -0.304853, 0.636946},
    {-0.722992, -0.412072, 0.554509},
    {-0.048151, -0.148191, 0.987786},
    {-0.262729, -0.30392, 0.915754},
    {-0.467354, -0.452586, 0.759438},
    {-0.089143, -0.274349, 0.957489},
    {0.033908, -0.400305, 0.915754},
    {-0.310547, -0.451877, 0.836282},
    {-0.187593, -0.577344, 0.794659},
    {-0.014375, -0.548111, 0.836282},
    {0.112066, -0.640855, 0.759438},
    {-0.508743, -0.579205, 0.636947},
    {-0.385367, -0.704925, 0.59546},
    {-0.228689, -0.703819, 0.672562},
    {-0.102585, -0.796808, 0.595461},
    {0.071125, -0.767619, 0.636948},
    {0.168486, -0.814943, 0.554509},
    {0.12606, -0.091587, 0.987786},
    {0.207859, -0.343786, 0.915754},
    {0.286017, -0.584336, 0.759438},
    {0.233376, -0.169558, 0.957489},
    {0.391194, -0.09145, 0.915753},
    {0.333798, -0.434985, 0.836282},
    {0.49112, -0.35682, 0.794658},
    {0.516845, -0.183046, 0.836281},
    {0.644124, -0.091451, 0.759434},
    {0.393647, -0.662827, 0.636948},
    {0.551342, -0.584336, 0.59546},
    {0.598705, -0.434986, 0.672561},
    {0.726114, -0.343786, 0.595458},
    {0.752031, -0.16956, 0.636945},
    {0.827126, -0.091587, 0.554505},
    {0.905035, 0.148193, 0.398685},
    {0.936571, 0.303922, 0.174544},
    {0.888267, 0.45259, -0.078388},
    {0.896269, 0.274349, 0.348473},
    {0.803908, 0.400308, 0.439871},
    {0.886873, 0.45188, 0.096235},
    {0.794655, 0.577349, 0.187594},
    {0.754418, 0.548114, 0.361145},
    {0.629137, 0.640858, 0.439872},
    {0.797217, 0.579207, -0.170187},
    {0.704932, 0.70493, -0.078388},
    {0.703826, 0.703824, 0.096235},
    {0.578466, 0.796813, 0.174546},
    {0.537886, 0.767622, 0.348475},
    {0.420608, 0.814945, 0.39869},
    {0.138726, 0.906533, 0.398689},
    {0.000363, 0.984649, 0.174545},
    {-0.155953, 0.984649, -0.078389},
    {0.016035, 0.93718, 0.348476},
    {-0.1323, 0.888262, 0.439872},
    {-0.155708, 0.983104, 0.096236},
    {-0.303533, 0.934171, 0.187594},
    {-0.288163, 0.886868, 0.361146},
    {-0.415083, 0.796378, 0.439872},
    {-0.304507, 0.937183, -0.170186},
    {-0.452594, 0.888265, -0.078389},
    {-0.451883, 0.886871, 0.096236},
    {-0.579061, 0.796381, 0.174546},
    {-0.563838, 0.748767, 0.348476},
    {-0.645086, 0.651853, 0.398689},
    {-0.819296, 0.412071, 0.398687},
    {-0.936345, 0.304618, 0.174544},
    {-0.98465, 0.155951, -0.078388},
    {-0.886357, 0.304854, 0.348476},
    {-0.885671, 0.148666, 0.43987},
    {-0.983104, 0.155706, 0.096236},
    {-0.982247, -0.000001, 0.187592},
    {-0.932509, 0, 0.361146},
    {-0.885671, -0.148666, 0.43987},
    {-0.985412, 0, -0.170183},
    {-0.984649, -0.155951, -0.078388},
    {-0.983104, -0.155707, 0.096237},
    {-0.936345, -0.304618, 0.174544},
    {-0.886357, -0.304854, 0.348476},
    {-0.819296, -0.412072, 0.398687},
    {-0.645086, -0.651853, 0.398689},
    {-0.579061, -0.796381, 0.174546},
    {-0.452594, -0.888265, -0.078389},
    {-0.563838, -0.748767, 0.348476},
    {-0.415083, -0.796379, 0.439871},
    {-0.451883, -0.886871, 0.096236},
    {-0.303534, -0.934171, 0.187593},
    {-0.288164, -0.886868, 0.361146},
    {-0.1323, -0.888262, 0.439871},
    {-0.304507, -0.937183, -0.170186},
    {-0.155953, -0.984649, -0.078388},
    {-0.155709, -0.983104, 0.096235},
    {0.000363, -0.984649, 0.174545},
    {0.016034, -0.937181, 0.348476},
    {0.138726, -0.906533, 0.398689},
    {0.420608, -0.814945, 0.398689},
    {0.578465, -0.796814, 0.174546},
    {0.704932, -0.70493, -0.078389},
    {0.537887, -0.767622, 0.348475},
    {0.629137, -0.640859, 0.439871},
    {0.703826, -0.703823, 0.096234},
    {0.794655, -0.577349, 0.187592},
    {0.754418, -0.548114, 0.361144},
    {0.803908, -0.400308, 0.439869},
    {0.797217, -0.579207, -0.170188},
    {0.888267, -0.45259, -0.078388},
    {0.886873, -0.451881, 0.096233},
    {0.936571, -0.303923, 0.174544},
    {0.896269, -0.27435, 0.348473},
    {0.905035, -0.148193, 0.398685},
    {0.294548, 0.906533, 0.302389},
    {0.452594, 0.888265, 0.078389},
    {0.579061, 0.796381, -0.174545},
    {0.304507, 0.937183, 0.170186},
    {0.155953, 0.984649, 0.078388},
    {0.451884, 0.886871, -0.096236},
    {0.303534, 0.934171, -0.187594},
    {0.155708, 0.983104, -0.096236},
    {-0.000363, 0.984649, -0.174545},
    {0.563838, 0.748766, -0.348476},
    {0.415083, 0.796378, -0.439872},
    {0.288163, 0.886868, -0.361147},
    {0.1323, 0.888262, -0.439872},
    {-0.016035, 0.93718, -0.348476},
    {-0.138726, 0.906533, -0.398689},
    {-0.771146, 0.560265, 0.302387},
    {-0.704932, 0.70493, 0.078389},
    {-0.578465, 0.796814, -0.174546},
    {-0.797217, 0.579207, 0.170187},
    {-0.888267, 0.45259, 0.078388},
    {-0.703826, 0.703824, -0.096235},
    {-0.794654, 0.57735, -0.187594},
    {-0.886873, 0.45188, -0.096234},
    {-0.93657, 0.303924, -0.174545},
    {-0.537885, 0.767622, -0.348476},
    {-0.629137, 0.640858, -0.439872},
    {-0.754417, 0.548115, -0.361144},
    {-0.803908, 0.400308, -0.43987},
    {-0.896269, 0.274351, -0.348473},
    {-0.905035, 0.148193, -0.398685},
    {-0.771146, -0.560265, 0.302387},
    {-0.888267, -0.45259, 0.078388},
    {-0.936571, -0.303924, -0.174545},
    {-0.797217, -0.579207, 0.170187},
    {-0.704932, -0.70493, 0.078389},
    {-0.886873, -0.45188, -0.096234},
    {-0.794654, -0.57735, -0.187594},
    {-0.703826, -0.703824, -0.096235},
    {-0.578465, -0.796814, -0.174546},
    {-0.896269, -0.274351, -0.348473},
    {-0.803908, -0.400308, -0.439871},
    {-0.754417, -0.548115, -0.361145},
    {-0.629137, -0.640858, -0.439872},
    {-0.537885, -0.767622, -0.348476},
    {-0.420608, -0.814945, -0.398689},
    {0.294548, -0.906533, 0.302388},
    {0.155953, -0.984649, 0.078388},
    {-0.000363, -0.984649, -0.174545},
    {0.304507, -0.937183, 0.170186},
    {0.452594, -0.888265, 0.078389},
    {0.155708, -0.983104, -0.096236},
    {0.303534, -0.934171, -0.187594},
    {0.451884, -0.886871, -0.096236},
    {0.579061, -0.796381, -0.174545},
    {-0.016035, -0.937181, -0.348476},
    {0.1323, -0.888262, -0.439872},
    {0.288164, -0.886868, -0.361147},
    {0.415083, -0.796378, -0.439872},
    {0.563838, -0.748766, -0.348476},
    {0.645086, -0.651853, -0.398689},
    {0.953186, 0, 0.302386},
    {0.984649, -0.155951, 0.078389},
    {0.936345, -0.304618, -0.174544},
    {0.985412, 0, 0.170184},
    {0.984649, 0.155951, 0.078389},
    {0.983104, -0.155706, -0.096236},
    {0.982247, 0, -0.187592},
    {0.983104, 0.155706, -0.096236},
    {0.936345, 0.304618, -0.174544},
    {0.886357, -0.304854, -0.348476},
    {0.885671, -0.148666, -0.43987},
    {0.932509, 0, -0.361146},
    {0.885671, 0.148666, -0.43987},
    {0.886357, 0.304854, -0.348476},
    {0.819296, 0.412071, -0.398687},
    {0.615326, 0.560263, -0.554509},
    {0.467353, 0.452587, -0.759438},
    {0.262728, 0.303921, -0.915754},
    {0.508743, 0.579205, -0.636947},
    {0.385368, 0.704925, -0.59546},
    {0.310547, 0.451878, -0.836282},
    {0.187593, 0.577345, -0.794658},
    {0.228688, 0.70382, -0.672562},
    {0.102585, 0.796808, -0.59546},
    {0.089144, 0.27435, -0.957489},
    {-0.033908, 0.400305, -0.915754},
    {0.014375, 0.548113, -0.836281},
    {-0.112066, 0.640855, -0.759438},
    {-0.071126, 0.767619, -0.636947},
    {-0.168486, 0.814943, -0.554509},
    {-0.342697, 0.758339, -0.554509},
    {-0.286018, 0.584335, -0.759438},
    {-0.20786, 0.343785, -0.915754},
    {-0.393648, 0.662826, -0.636948},
    {-0.551342, 0.584336, -0.59546},
    {-0.333798, 0.434984, -0.836282},
    {-0.491122, 0.356818, -0.794657},
    {-0.598706, 0.434985, -0.672561},
    {-0.726114, 0.343786, -0.595458},
    {-0.233376, 0.169558, -0.957489},
    {-0.391194, 0.09145, -0.915753},
    {-0.516846, 0.183045, -0.83628},
    {-0.644125, 0.09145, -0.759434},
    {-0.752031, 0.169559, -0.636945},
    {-0.827125, 0.091587, -0.554505},
    {-0.827125, -0.091587, -0.554505},
    {-0.644124, -0.091451, -0.759434},
    {-0.391194, -0.091451, -0.915753},
    {-0.752031, -0.169559, -0.636945},
    {-0.726114, -0.343786, -0.595458},
    {-0.516846, -0.183046, -0.83628},
    {-0.491122, -0.35682, -0.794656},
    {-0.598706, -0.434986, -0.672561},
    {-0.551343, -0.584337, -0.595459},
    {-0.233377, -0.169558, -0.957489},
    {-0.20786, -0.343785, -0.915754},
    {-0.333798, -0.434986, -0.836281},
    {-0.286018, -0.584335, -0.759438},
    {-0.393647, -0.662827, -0.636947},
    {-0.342697, -0.758339, -0.554509},
    {0.722992, 0.412072, -0.554509},
    {0.78951, 0.148666, -0.59546},
    {0.78951, -0.148666, -0.595459},
    {0.708071, 0.304853, -0.636946},
    {0.574859, 0.304617, -0.759438},
    {0.740043, 0, -0.67256},
    {0.607058, 0, -0.794658},
    {0.525729, 0.155706, -0.836281},
    {0.370234, 0.15595, -0.915754},
    {0.708071, -0.304854, -0.636946},
    {0.574859, -0.304617, -0.759438},
    {0.525729, -0.155706, -0.83628},
    {0.370234, -0.15595, -0.915754},
    {0.288471, 0, -0.957489},
    {0.155817, 0, -0.987786},
    {-0.168486, -0.814943, -0.554509},
    {-0.112065, -0.640855, -0.759438},
    {-0.033907, -0.400305, -0.915754},
    {-0.071125, -0.767619, -0.636948},
    {0.102585, -0.796808, -0.595461},
    {0.014375, -0.548112, -0.836282},
    {0.187595, -0.577345, -0.794658},
    {0.228689, -0.703819, -0.672562},
    {0.385367, -0.704925, -0.59546},
    {0.089144, -0.27435, -0.957489},
    {0.262729, -0.30392, -0.915754},
    {0.310548, -0.451878, -0.836281},
    {0.467353, -0.452587, -0.759438},
    {0.508744, -0.579205, -0.636947},
    {0.615326, -0.560263, -0.554509}};
      
}
