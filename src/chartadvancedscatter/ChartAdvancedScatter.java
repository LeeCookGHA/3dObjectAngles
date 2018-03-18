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
    final double chartHorMin = -20d;
    final double chartHorMax =  20d;

    final double chartVirMin = -20d;
    final double chartVirMax =  20d;

    // Photo Diode Info
    final static double maxAoI = 90d;
    final static double relPowerRatioVsAoI = 1d/maxAoI;  // Approx to linear 0.9==25deg
    final static double maxDist = 20d;
    final static double relPowerRatioVsDist = 1d/maxDist; // Approx drops to zero at 20m
    final static double minRelPower = 0.05;
    
    // Sensor Cluster Info
    static int CLUSTER_SIZE = 32;
    Vector3 clusterOriginPosition = new Vector3(0.5, 0, 0); // meters
//    final static double clusterAccelPitch = 0d;
//    final static double clusterAccelRoll = 0d;
//    final static double clusterHeading = 0d;
//    Vector3 clusterPointingVector = new Vector3(1,0,0);
    Vector3[] clusterSensorPositions = new Vector3[CLUSTER_SIZE]; // Meters
    Vector3[] clusterSensorNormalPositions = new Vector3[CLUSTER_SIZE]; // Meters
//    Vector3[] clusterSensorTransformedPositions = new Vector3[CLUSTER_SIZE]; // Meters
//    Vector3[] clusterSensorTransformedNormalPositions = new Vector3[CLUSTER_SIZE]; // Meters
    
    // Base1 info
    Vector3 base1OriginPosition = new Vector3(0,0,0); // meters
//    final static double base1AccelPitch = 0d;
//    final static double base1AccelRoll = 0d;
//    final static double base1Heading = -90d;
//    Vector3 base1PointingVector = new Vector3(1,0,0);
    
    // Base to Sensor relative info
    Vector3 base1ToClusterVector = new Vector3();
    Spherical3 base1ToClusterSpherical = new Spherical3();
    Spherical3[] base1ToSensorSpherical = new Spherical3[CLUSTER_SIZE];
    Vector3[] base1ToSensorVector = new Vector3[CLUSTER_SIZE];
    double[] base1ToSensorAoI = new double[CLUSTER_SIZE];
    double[] base1ToSensorRelativePower = new double[CLUSTER_SIZE];
    boolean[] base1ToSensorVisible = new boolean[CLUSTER_SIZE];
    
    // Test objects
//    Vector3 testVector = new Vector3(1,0,1); // meters
//    Spherical3 testSpherical = new Spherical3();


    // Initialise the cluster
    private void init(Stage primaryStage) {
        base1ToClusterVector.set(1.0852, 0.0171, 0.0464);
        base1ToClusterSpherical.setFromVector3(base1ToClusterVector);
        
        // Setup sensor cluster detector positions and normal positions (wrt to the detector)
        for (int count = 0; count < CLUSTER_SIZE; count++) {
            clusterSensorPositions[count] = new Vector3();
            clusterSensorPositions[count].x = devicePoints[count][2];
            clusterSensorPositions[count].y = devicePoints[count][0];
            clusterSensorPositions[count].z = devicePoints[count][1];

            clusterSensorNormalPositions[count] = new Vector3();
            clusterSensorNormalPositions[count].x = clusterSensorPositions[count].x + deviceNormals[count][2]; // Meters
            clusterSensorNormalPositions[count].y = clusterSensorPositions[count].y + deviceNormals[count][0]; // Meters
            clusterSensorNormalPositions[count].z = clusterSensorPositions[count].z + deviceNormals[count][1]; // Meters
            
            // Transform for orientation
clusterSensorPositions[count].x *= -1d;
clusterSensorPositions[count].y *= -1d;
clusterSensorNormalPositions[count].x *= -1d;
clusterSensorNormalPositions[count].y *= -1d;
            
            // Transform for cluster origin position in base relative terms
            clusterSensorPositions[count].add(clusterOriginPosition);
            clusterSensorNormalPositions[count].add(clusterOriginPosition);
            
            // Get the Spherical Co-ords for the sensors
            base1ToSensorVector[count] = new Vector3();
            base1ToSensorVector[count] = clusterSensorPositions[count].sub(base1OriginPosition);
            base1ToSensorSpherical[count] = new Spherical3();
            base1ToSensorSpherical[count].setFromVector3(base1ToSensorVector[count]);
            
            // work out the angle to the base from each sensor normal
            base1ToSensorAoI[count] = angleThreePoints(base1OriginPosition, clusterSensorPositions[count], clusterSensorNormalPositions[count]);
            System.out.printf("\n\r   %d: %3.4f", count, Math.toDegrees(base1ToSensorAoI[count]));
            
            base1ToSensorVisible[count] = (Math.toDegrees(base1ToSensorAoI[count]) < 80d);
        }
        

        // Run through the cluster and print the results
        System.out.printf("\n\rBASE 1 (x:%2.4f, y:%2.4f, z:%2.4f)", base1OriginPosition.x, base1OriginPosition.y, base1OriginPosition.z);
        System.out.print("\n\r   Sens: (Base1 centric x, y, z), (Az Angle, El Angle), AoI, Relative Power, VISIBLE?");
        for (int count = 0; count < CLUSTER_SIZE; count++) {
            System.out.printf("\n\r   %d: ", count);
//            System.out.printf("(%2.4f, %2.4f, %2.4f), ", base1ToSensorVector[count].x, base1ToSensorVector[count].y, base1ToSensorVector[count].z);
            System.out.printf("(%3.4f, %3.4f), ", Math.toDegrees(base1ToSensorSpherical[count].az), Math.toDegrees(base1ToSensorSpherical[count].el));
//            System.out.printf("%.3f, %.3f, ", base1ToSensorAoI[count], base1ToSensorRelativePower[count]);

            if(base1ToSensorVisible[count]){
                System.out.print("YES");
            } else {
                System.out.print("NO");
            }
        }
        System.out.print("\n\r");

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

        // add starting data
        if (base1ToSensorVisible[1] || base1ToSensorVisible[9]) {
            XYChart.Series<Number, Number> seriesS1 = new XYChart.Series<Number, Number>();
            seriesS1.setName("Right");
            if (base1ToSensorVisible[ 1]) seriesS1.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[1].az), Math.toDegrees(base1ToSensorSpherical[1].el)));
            if (base1ToSensorVisible[ 9]) seriesS1.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[9].az), Math.toDegrees(base1ToSensorSpherical[9].el)));
            sc.getData().add(seriesS1);
        }

        if (base1ToSensorVisible[22] || base1ToSensorVisible[30]) {
            XYChart.Series<Number, Number> seriesS2 = new XYChart.Series<Number, Number>();
            seriesS2.setName("Left");
            if (base1ToSensorVisible[22]) seriesS2.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[22].az), Math.toDegrees(base1ToSensorSpherical[22].el)));
            if (base1ToSensorVisible[30]) seriesS2.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[30].az), Math.toDegrees(base1ToSensorSpherical[30].el)));
            sc.getData().add(seriesS2);
        }

        XYChart.Series<Number, Number> seriesEdges = new XYChart.Series<Number, Number>();
        seriesEdges.setName("Sensors");
        if (base1ToSensorVisible[ 0]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 0].az), Math.toDegrees(base1ToSensorSpherical[ 0].el)));
//        if (base1ToSensorVisible[ 1]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 1].az), Math.toDegrees(base1ToSensorSpherical[ 1].el)0);
        if (base1ToSensorVisible[ 2]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 2].az), Math.toDegrees(base1ToSensorSpherical[ 2].el)));
        if (base1ToSensorVisible[ 3]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 3].az), Math.toDegrees(base1ToSensorSpherical[ 3].el)));
        if (base1ToSensorVisible[ 4]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 4].az), Math.toDegrees(base1ToSensorSpherical[ 4].el)));
        if (base1ToSensorVisible[ 5]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 5].az), Math.toDegrees(base1ToSensorSpherical[ 5].el)));
        if (base1ToSensorVisible[ 6]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 6].az), Math.toDegrees(base1ToSensorSpherical[ 6].el)));
        if (base1ToSensorVisible[ 7]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 7].az), Math.toDegrees(base1ToSensorSpherical[ 7].el)));
        if (base1ToSensorVisible[ 8]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 8].az), Math.toDegrees(base1ToSensorSpherical[ 8].el)));
//        if (base1ToSensorVisible[ 9]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[ 9].az), Math.toDegrees(base1ToSensorSpherical[ 9].el)));
        if (base1ToSensorVisible[10]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[10].az), Math.toDegrees(base1ToSensorSpherical[10].el)));
        if (base1ToSensorVisible[11]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[11].az), Math.toDegrees(base1ToSensorSpherical[11].el)));
        if (base1ToSensorVisible[12]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[12].az), Math.toDegrees(base1ToSensorSpherical[12].el)));
        if (base1ToSensorVisible[13]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[13].az), Math.toDegrees(base1ToSensorSpherical[13].el)));
        if (base1ToSensorVisible[14]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[14].az), Math.toDegrees(base1ToSensorSpherical[14].el)));
        if (base1ToSensorVisible[15]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[15].az), Math.toDegrees(base1ToSensorSpherical[15].el)));
        if (base1ToSensorVisible[16]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[16].az), Math.toDegrees(base1ToSensorSpherical[16].el)));
        if (base1ToSensorVisible[17]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[17].az), Math.toDegrees(base1ToSensorSpherical[17].el)));
        if (base1ToSensorVisible[18]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[18].az), Math.toDegrees(base1ToSensorSpherical[18].el)));
        if (base1ToSensorVisible[19]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[19].az), Math.toDegrees(base1ToSensorSpherical[19].el)));
        if (base1ToSensorVisible[20]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[20].az), Math.toDegrees(base1ToSensorSpherical[20].el)));
        if (base1ToSensorVisible[21]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[21].az), Math.toDegrees(base1ToSensorSpherical[21].el)));
//        if (base1ToSensorVisible[22]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[22].az), Math.toDegrees(base1ToSensorSpherical[22].el)));
        if (base1ToSensorVisible[23]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[23].az), Math.toDegrees(base1ToSensorSpherical[23].el)));
        if (base1ToSensorVisible[24]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[24].az), Math.toDegrees(base1ToSensorSpherical[24].el)));
        if (base1ToSensorVisible[25]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[25].az), Math.toDegrees(base1ToSensorSpherical[25].el)));
        if (base1ToSensorVisible[26]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[26].az), Math.toDegrees(base1ToSensorSpherical[26].el)));
        if (base1ToSensorVisible[27]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[27].az), Math.toDegrees(base1ToSensorSpherical[27].el)));
        if (base1ToSensorVisible[28]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[28].az), Math.toDegrees(base1ToSensorSpherical[28].el)));
        if (base1ToSensorVisible[29]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[29].az), Math.toDegrees(base1ToSensorSpherical[29].el)));
//        if (base1ToSensorVisible[30]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[30].az), Math.toDegrees(base1ToSensorSpherical[30].el)));
        if (base1ToSensorVisible[31]) seriesEdges.getData().add(new XYChart.Data<Number, Number>(Math.toDegrees(base1ToSensorSpherical[31].az), Math.toDegrees(base1ToSensorSpherical[31].el)));
        sc.getData().add(seriesEdges);

        return sc;
    }
    
    
        double angleThreePoints(Vector3 Start, Vector3 Centre, Vector3 End)
    {
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
        init(primaryStage);
        primaryStage.show();
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

}
