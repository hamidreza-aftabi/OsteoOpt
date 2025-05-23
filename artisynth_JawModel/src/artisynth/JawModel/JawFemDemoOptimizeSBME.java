package artisynth.JawModel;

import java.awt.Color;
import java.awt.GridBagConstraints;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSeparator;
import javax.swing.SwingConstants;

import com.github.sardine.model.Set;

import artisynth.core.femmodels.FemCutPlane;
import artisynth.core.femmodels.FemElement;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.fields.ScalarNodalField;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.TrackingController;
import artisynth.core.materials.LinearElasticContact;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.PointAttachable;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.CollisionBehavior.Method;
import artisynth.core.mechmodels.CollisionManager.ColliderType;
import artisynth.core.mechmodels.MechSystemSolver.PosStabilization;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.renderables.ColorBar;
import artisynth.core.util.ArtisynthIO;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.ScalarRange;
import artisynth.core.workspace.RootModel;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Matrix3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;
import maspack.render.color.ColorMapBase;
import maspack.render.color.RainbowColorMap;
import maspack.util.DoubleInterval;
import maspack.util.PathFinder;
import maspack.widgets.IntegerField;


public class JawFemDemoOptimizeSBME extends RootModel implements ActionListener {
  
   
   JawModelFEM myJawModel; 
   TrackingController myTrackingController;
   ScalarNodalField integField;
   
   boolean Element_Visibility = false;
   
   boolean myShowDonorStress = false;
   

  
   
   boolean myUseScrews = true;
  
   
   public static double DENSITY_TO_mmKS = 1e-9; // convert density from MKS tp mmKS
   public static double PRESSURE_TO_mmKS = 1e-3; // convert pressure from MKS tp mmKS

   public static double CancellousBoneDensity = 100.0 * DENSITY_TO_mmKS;
   public static double CancellousBoneE = 1.3*1e9 * PRESSURE_TO_mmKS;
   public static double CancellousBoneNu = 0.3;

   public static double myTitaniumDensity = 4420.0 * DENSITY_TO_mmKS;
   public static double myTitaniumE = 100*1e9 * PRESSURE_TO_mmKS;
   public static double myTitaniumNu = 0.3;
   
   public static double corticalBoneYoungModulus =  13.7*1e9 * PRESSURE_TO_mmKS;
   public static double corticalBonePoissonRatio = 0.3;
   public static double corticalBoneDensity = 2000.0 * DENSITY_TO_mmKS;
 
   public static double corticalAppositionDensity = 0.002;
   public static double cancellousAppositionDensity = 0.00015;

   
   double t=0.75; 

   protected static double unitConversion = 1000;

   
   // for interaction between the donor check Optimization paper for prosthesis
   double  DEFAULT_E =0.03 * 1e9 * PRESSURE_TO_mmKS;
   double  DEFAULT_Thickness = .2; // mm   
   double  DEFAULT_Damping = 10; 
   double  DEFAULT_Nu = 0.3;
   
   
   
   RigidBody myDonor0;
   FemModel3d myPlate;
 
   
   
   RigidBody myMandibleRight;
   RigidBody myMandibleLeft;
   RigidBody myDonor0Mesh;

   
   PolygonalMesh donorMeshSurface;
   PolygonalMesh surfaceLeft;
   PolygonalMesh surfaceRight;

   
  

   private static Color PALE_BLUE = new Color (0.6f, 0.6f, 1.0f);
   private static Color GOLD = new Color (1f, 0.8f, 0.1f);

   String myGeoDir = PathFinder.getSourceRelativePath (
      JawFemDemoOptimizeSBME.class, "geometry/");
   
   
   ArrayList<String> MuscleAbbreviation = new ArrayList<String>();

 

   protected String workingDirname = "data/";
   String probesFilename ;

   HashMap<String,String> condyleMusclesLeft = new HashMap<String,String>();
   HashMap<String,String> condyleMusclesRight = new HashMap<String,String>();

   HashMap<String,String> ramusMusclesLeft = new HashMap<String,String>();
   HashMap<String,String> ramusMusclesRight = new HashMap<String,String>();

   HashMap<String,String> bodyMusclesLeft = new HashMap<String,String>();
   HashMap<String,String> bodyMusclesRight = new HashMap<String,String>();

   HashMap<String,String> hemisymphysisMusclesLeft = new HashMap<String,String>();
   HashMap<String,String> hemisymphysisMusclesRight = new HashMap<String,String>();
   
   
   HashSet<FemNode3d> nodesOnSurfaceLeft =   new HashSet<FemNode3d>();
   HashSet<FemElement3d> elemsNearSurfaceLeft =  new HashSet<FemElement3d>();
   
   HashSet<FemNode3d> nodesOnSurfaceRight =   new HashSet<FemNode3d>();
   HashSet<FemElement3d> elemsNearSurfaceRight =  new HashSet<FemElement3d>();
   HashSet<FemElement3d> elementsCloseToSurface = new HashSet<FemElement3d>();
   
   JFrame frame;
   JPanel panel; 
   JSeparator seperator1;
   JCheckBox cb1,cb2,cb3,cb4,cb5,cb6,cb7,cb8;      
   GridBagConstraints gc;
   JLabel label;
   JButton button;
   
   
   public static PropertyList myProps = new PropertyList (JawFemDemoOptimizeSBME.class, RootModel.class);
  
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   
 
 

/*
   @Override
   public void prerender ( RenderList list ) {
     super.prerender (list );
     ColorBar cbar = ( ColorBar)( renderables ().get("colorBar"));
     cbar.setColorMap (integField. getColorMap ());
     cbar.updateLabels (integField.getValueRange ().getLowerBound (), integField.getValueRange().getUpperBound());
   }
   */


   public JawFemDemoOptimizeSBME () {
      
   }

   
   public JawFemDemoOptimizeSBME (String name){
      super(null);
   }
   
   
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      
 
     
      setWorkingDir();
      myJawModel = new JawModelFEM("jawmodel");
      addModel (myJawModel);
      getRoot (this).setMaxStepSize (0.001);
      
    
      myJawModel.setStabilization (
         PosStabilization.GlobalStiffness); // more accurate stabilization
      
      
 
      //addClosingForce ();
      addOpening();
      
      addFemDonorPlate();
     
      
      for (double i=0.01; i<=2*t; i=i+0.01 ){
         addWayPoint (i);
      }
      addBreakPoint (t);    
      
      loadProbes("probe.art");
     
      //addControlPanel();
      
      loadBoluses();
            
     

      
      condyleMusclesLeft.put("lip","Left Inferior Lateral Pterygoid");
      condyleMusclesLeft.put("lsp","Left Superior Lateral Pterygoid");
      
      condyleMusclesRight.put("rip","Right Inferior Lateral Pterygoid");
      condyleMusclesRight.put("rsp","Right Superior Lateral Pterygoid");


      ramusMusclesLeft.put("lpt", "Left Posterior Temporal");
      ramusMusclesLeft.put("lmt", "Left Middle Temporal");
      ramusMusclesLeft.put("lat", "Left Anterior Temporal");
      ramusMusclesLeft.put("ldm", "Left Deep Masseter");
      ramusMusclesLeft.put("lsm", "Left Superficial Masseter");
      ramusMusclesLeft.put("lmp", "Left Medial Pterygoid");
      
      ramusMusclesRight.put("rpt", "Right Posterior Temporal");
      ramusMusclesRight.put("rmt", "Right Middle Temporal");
      ramusMusclesRight.put("rat", "Right Anterior Temporal");
      ramusMusclesRight.put("rdm", "Right Deep Masseter");
      ramusMusclesRight.put("rsm", "Right Superficial Masseter");
      ramusMusclesRight.put("rmp", "Right Medial Pterygoid");
      
      
      bodyMusclesLeft.put("lpm","Left Posterior Mylohyoid");
      bodyMusclesLeft.put("lam","Left Mylohyoid");

      bodyMusclesRight.put("ram","Right Mylohyoid");
      bodyMusclesRight.put("rpm","Right Posterior Mylohyoid");


      hemisymphysisMusclesLeft.put("lad", "Left Anterior Digastric" );
      hemisymphysisMusclesLeft.put("lgh", "Left Geniohyoid" );        
                      
      hemisymphysisMusclesRight.put("rad", "Right Anterior Digastric" );
      hemisymphysisMusclesRight.put("rgh", "Right Geniohyoid" );      

      
      frame = new JFrame();
      panel = new JPanel();
      gc = new GridBagConstraints();
  
      gc.anchor = GridBagConstraints.EAST;
      gc.fill = GridBagConstraints.NONE;
  
      seperator1 = new JSeparator();
  
      cb1 = new JCheckBox("Left Condyle Defect (Left C)");
      cb2 = new JCheckBox("Right Condyle Defect (Right C)");

      cb3 = new JCheckBox("Left Ramus Defect (Left R)");
      cb4 = new JCheckBox("Right Ramus Defect (Right R)");
  
      cb5 = new JCheckBox("Left Body Defect (Left B)");
      cb6 = new JCheckBox("Right Body Defect (Right B)");
  
      cb7 = new JCheckBox("Left HemiSymphysis Defect (Left SH)");
      cb8 = new JCheckBox("Right HemiSymphysis Defect (Right SH)");
  
      button = new JButton("Initialize/Reset");
  
  
      cb1.addActionListener(this);
      cb2.addActionListener(this);
      cb3.addActionListener(this);
      cb4.addActionListener(this);
      cb5.addActionListener(this);
      cb6.addActionListener(this);
      cb7.addActionListener(this);
      cb8.addActionListener(this);

      button.addActionListener(this);
  
      gc.gridx = 0;
      gc.gridy = 1;
      panel.add(cb1,gc);
  
      gc.gridx = 0;
      gc.gridy = 2;
      panel.add(cb2,gc);
  
      gc.gridx = 0;
      gc.gridy = 3;
      panel.add(cb3,gc);
  
      gc.gridx = 0;
      gc.gridy = 4;
      panel.add(cb4,gc);
  
      gc.gridx = 0;
      gc.gridy = 5;
      panel.add(cb5,gc);
  
      gc.gridx = 0;
      gc.gridy = 6;
      panel.add(cb6,gc);
  
      gc.gridx = 0;
      gc.gridy = 7;
      panel.add(cb7,gc);
  
      gc.gridx = 0;
      gc.gridy = 8;
      panel.add(cb8,gc);
  
      seperator1.setOrientation(SwingConstants.HORIZONTAL);
      gc.gridx = 0;
      gc.gridy = 9;
      panel.add(seperator1,gc);
  
      gc.gridx = 0;
      gc.gridy = 10;
      panel.add(button,gc);
  
     panel.setLayout(new GridLayout(0,1));

     frame.setTitle("Urken's Defect Classification (Forward)");
     frame.setSize(330, 500);
     //frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
     frame.add(panel);
     frame.setVisible(false);
    
     
     //just since imported twice make one invisble 
     myDonor0Mesh = (RigidBody)myJawModel.findComponent (
     "rigidBodies/donor_mesh0");
     
     RenderProps.setVisible (myDonor0Mesh, false);
   
   }
   
   
 
 
  
   public RigidBody createDonorModel (
      MechModel mech, String name, String meshName) {

      // load the triangular surface mesh and then call createFromMesh,
      // which uses tetgen to create a tetrahedral volumetric mesh:
      PolygonalMesh surface = loadMesh (meshName);
      
      // Create a rigid body from the loaded mesh
      RigidBody rigidBody = RigidBody.createFromMesh(
         "donor", surface, /*density=*/corticalBoneDensity, /*scale=*/1.0);
      
      rigidBody.setDynamic(true); // Make it dynamic (movable)


      mech.add(rigidBody);
      return rigidBody;
   }

   

   
   /**
    * Load a polygonal mesh with the given name from the geometry folder.
    */
   private PolygonalMesh loadMesh (String meshName) {
      PolygonalMesh mesh = null;
      String meshPath = myGeoDir + meshName;
      try {
         mesh = new PolygonalMesh (meshPath);
      }
      catch (IOException e) {
         System.out.println ("Can't open or load "+meshPath);
      }
      mesh.transform (myJawModel.amiraTranformation);
      return mesh;
   }

   
   /**
    * Attach an FEM model to another body (either an FEM or a rigid body)
    * by attaching a subset of its nodes to that body.
    *
    * @param mech MechModel containing all the components
    * @param fem FEM model to be connected
    * @param body body to attach the FEM to. Can be a rigid body
    * or another FEM.
    * @param nodeNums numbers of the FEM nodes which should be attached
    * to the body
    */
   public void attachFemToBody (
      MechModel mech, FemModel3d fem, PointAttachable body, int[] nodeNums) {

      for (int num : nodeNums) {
         mech.attachPoint (fem.getNodeByNumber(num), body);
      }
   }

  
   
   /**
    * Attach an FEM model to another body (either an FEM or a rigid body) by
    * attaching all surface nodes that are within a certain distance of the
    * body's surface mesh.
    *
    * @param mech MechModel containing all the components
    * @param fem FEM model to be connected
    * @param body body to attach the FEM to. Can be a rigid body
    * or another FEM.
    * @param dist distance to the body surface for attaching nodes
    */
   public void attachFemToBody (
      MechModel mech, FemModel3d fem, PointAttachable body, double dist) {
      
      PolygonalMesh surface = null;
      if (body instanceof RigidBody) {
         surface = ((RigidBody)body).getSurfaceMesh();
      }
      else if (body instanceof FemModel3d) {
         surface = ((FemModel3d)body).getSurfaceMesh();
      }
      else {
         throw new IllegalArgumentException (
            "body is neither a rigid body nor an FEM model");
      }
      for (FemNode3d n : fem.getNodes()) {
         if (fem.isSurfaceNode (n)) {
            double d = surface.distanceToPoint (n.getPosition());
            if (d < dist) {
               mech.attachPoint (n, body);
               // set the attached points to render as red spheres
               RenderProps.setSphericalPoints (n, 0.5, Color.RED);
            }
         }
      }
   }

   
   
   
   public void addFemDonorPlate() {

      
      myDonor0 = createDonorModel (
         myJawModel, "donor0", "donor_opt0_remeshed.obj");
      
      //plate
      
      //String platePath = myGeoDir + "plate_final_case4_inf.art";
      String platePath = myGeoDir + "plate_opt.art";
   
      try {
         // read the FEM using the loadComponent utility
         myPlate = ComponentUtils.loadComponent (
            platePath, null, FemModel3d.class);
         // set the material properties to correspond to titanium 
         myPlate.setName ("plate");
         myPlate.setDensity (myTitaniumDensity);
         myPlate.setMaterial (new LinearMaterial (myTitaniumE, myTitaniumNu));
         myPlate.setMassDamping (10.0);
         myPlate.setStiffnessDamping (0.0);
         
         // set render properties for the plate
         RenderProps.setFaceColor (myPlate, GOLD);
         RenderProps.setPointRadius (myPlate, 0.5);         
         Vector3d translation = new Vector3d(7.0, -40.957,  -53.909);
         
         myPlate.transformPose (new RigidTransform3d (translation, AxisAngle.IDENTITY));
         myPlate.transformPose (myJawModel.amiraTranformation);
      }
      catch (IOException e) {
         System.out.println ("Can't open or load "+platePath);
         e.printStackTrace(); 
      }
      myJawModel.addModel (myPlate);
      
      //attach the plate to the left and right mandible segments. We use
      // explicitly defined nodes to do this, since the plate may be some
      // distance from the segments.
           
      myMandibleRight = (RigidBody)myJawModel.findComponent (
      "rigidBodies/jaw_resected");
      
      myMandibleLeft = (RigidBody)myJawModel.findComponent (
      "rigidBodies/jaw");

   
      
      
      int[] leftAttachNodes = {0,1,4,5};
      
      attachFemToBody (myJawModel, myPlate, myMandibleLeft, leftAttachNodes);
      
      
      int numNodes = myPlate.numNodes (); 
      
      int[] rightAttachNodes = {numNodes-3, numNodes-4, numNodes-7, numNodes-8  };
     
      attachFemToBody (myJawModel, myPlate, myMandibleRight, rightAttachNodes);

      
      
      attachPlateToDonorSegments (myJawModel);

      
 
   }
   
   
   
   
   /**
    * Helper method to attach the plate to the donor segments.
    */
   private void attachPlateToDonorSegments (MechModel mech) {
         // attach plate to donor segments using rigid bodies representing screws

        double attachTol = 0.05;
        //int hexElem =9;
        
        
        RigidBody screw0 = (RigidBody)myJawModel.findComponent ("rigidBodies/screw0");
        
        int  hexElem0  = findClosestHexElementNumber(myPlate,screw0);
      
        //System.out.println (hexElem);

       
         attachElemToSegmentforRigid (
             mech, screw0,  (HexElement)myPlate.getElementByNumber(hexElem0),
             myDonor0, attachTol);
         
         screw0.setDensity (myTitaniumDensity);

      
   }
   
   

   public  int findClosestHexElementNumber( FemModel3d myPlate, RigidBody screw) {
      double minDistance = Double.MAX_VALUE;
      HexElement closestElement = null;
      Point3d screwPosition = screw.getPosition();  // Get the screw's position directly

      // Iterate through all element numbers in the plate
      for (int i = 0; i < myPlate.numElements(); i++) {
          HexElement elem = (HexElement) myPlate.getElement(i);

          // Calculate centroid of the element
          Point3d centroid = new Point3d();
          elem.computeCentroid(centroid);
          
          // Calculate distance to the screw
          double distance = centroid.distance(screwPosition);
          
          // Update the closest element if necessary
          if (distance < minDistance) {
              minDistance = distance;
              closestElement = elem;
          }
      }

      // If a closest element is found, return its number
      if (closestElement != null) {
          return closestElement.getNumber();
      } else {
          // Return -1 or any other indicator if no element is found
          return -1;
      }
  }


 
   private void attachElemToSegmentforRigid (
      MechModel mech, RigidBody screw, HexElement hex, RigidBody donor, double attachTol) {

      int nattach = 0;

      System.out.println ("screw attached attached with" + nattach + " points");
      // also attach the screw to the hex element
      mech.attachFrame (screw, hex);
      mech.attachFrame (donor, screw);

   }

   
   ///////// FOOD BOLUS
   
   public boolean bolusesLoaded = false;

   ArrayList<FoodBolus> myFoodBoluses = new ArrayList<FoodBolus>();

   protected double bolusDiameter = 8; // mm

   protected double bolusMaxResistance = 80; // N

   protected double bolusStiffness = bolusMaxResistance / (bolusDiameter);

   protected double bolusDamping = 0.01;

   
   public void loadBoluses() {
      if (bolusesLoaded) return;
      createBoluses();
      for (FoodBolus fb : myFoodBoluses) {
         myJawModel.addForceEffector(fb);
         // System.out.println(fb.getName() + " P = "
         // + fb.getPlane().toString("%8.2f"));
         if (fb.getName().equals("leftbolus")) fb.setActive(true);
         else
            fb.setActive(false);
      }
      bolusesLoaded = true;
      PlanarConnector LBITE = (PlanarConnector) myJawModel.bodyConnectors().get("LBITE");
      LBITE.setEnabled (true);
      RenderProps.setVisible (LBITE, true);
   }
   
   

   public void createBoluses() {
      // TODO: create bolus using occlusal plane angle
      //Point3d rightbitePos = myJawModel.frameMarkers().get("rbite").getLocation ();
      Point3d leftbitePos = myJawModel.frameMarkers().get("lbite")
            .getLocation();
      //createFoodBolus("rightbolus", rightbitePos, (PlanarConnector) myJawModel.bodyConnectors().get("RBITE"));
      createFoodBolus("leftbolus", leftbitePos, (PlanarConnector) myJawModel
            .bodyConnectors().get("LBITE"));
      updateBoluses();
   }
   
   public void updateBoluses() {
      System.out.println("bolus dirs updated");
      if (myFoodBoluses.size() >= 2) {
         //updateBolusDirection("RBITE", myFoodBoluses.get(0));
         updateBolusDirection("LBITE", myFoodBoluses.get(1));
      }
   }
   
   
   
   public void updateBolusDirection(String constraintName, FoodBolus bolus) {
      PlanarConnector bite = (PlanarConnector) myJawModel.bodyConnectors()
            .get(constraintName);
      if (bite != null && bolus != null) {
         bolus.setPlane(bite);
         // RigidTransform3d XPB = bite.getXDB();
         // // System.out.println(constraintName + " X =\n" +
         // XPB.toString("%8.2f"));
         // bolus.setPlane( getPlaneFromX (XPB));
         // // System.out.println(bolus.getName() + "plane =\n" +
         // bolus.myPlane.toString("%8.2f"));
      }
   }
   
   
   public void createFoodBolus(String bolusName, Point3d location,
      PlanarConnector plane) {
   FoodBolus fb = new FoodBolus(bolusName, plane, bolusDiameter,
         bolusMaxResistance, bolusDamping);

   RenderProps bolusPtProps = new RenderProps(myJawModel.getRenderProps());
   bolusPtProps.setPointRadius(0.0);
   bolusPtProps.setPointColor(Color.BLACK);

   RigidBody jaw = myJawModel.rigidBodies().get("jaw");
   FrameMarker bolusContactPt = new FrameMarker();
   myJawModel.addFrameMarker(bolusContactPt, jaw, location);
   bolusContactPt.setName(bolusName + "ContactPoint");
   bolusContactPt.setRenderProps(new RenderProps(bolusPtProps));

   fb.setCollidingPoint(bolusContactPt);
   myFoodBoluses.add(fb);
}

   
   
   @Override
   public void actionPerformed(ActionEvent event) {
                   
           
           checkBoxJob(condyleMusclesLeft, cb1);
           checkBoxJob(condyleMusclesRight, cb2);
           
           checkBoxJob(ramusMusclesLeft, cb3);
           checkBoxJob(ramusMusclesRight, cb4);

           checkBoxJob(bodyMusclesLeft, cb5);
           checkBoxJob(bodyMusclesRight, cb6);
           
           checkBoxJob(hemisymphysisMusclesLeft, cb7);
           checkBoxJob(hemisymphysisMusclesRight, cb8);
           
           
           if (event.getSource() == button) {
                   
                   
                   disableCorrMuscles(bodyMusclesLeft);
                   disableCorrMuscles(bodyMusclesRight);
                   disableCorrMuscles(condyleMusclesLeft);
                   disableCorrMuscles(condyleMusclesRight);
                   disableCorrMuscles(ramusMusclesLeft);
                   disableCorrMuscles(ramusMusclesRight);
                   disableCorrMuscles(hemisymphysisMusclesLeft);
                   disableCorrMuscles(hemisymphysisMusclesRight);
                   
                   
                   enableCorrMuscles(bodyMusclesLeft);
                   enableCorrMuscles(bodyMusclesRight);
                   enableCorrMuscles(condyleMusclesLeft);
                   enableCorrMuscles(condyleMusclesRight);
                   enableCorrMuscles(ramusMusclesLeft);
                   enableCorrMuscles(ramusMusclesRight);
                   enableCorrMuscles(hemisymphysisMusclesLeft);
                   enableCorrMuscles(hemisymphysisMusclesRight);

                   //myJawModel.assembleBilateralExcitors();
                   //myJawModel.assembleMuscleGroups();
                   //loadProbes("adapted11_l.art");
                   cb1.setSelected(false);
                   cb2.setSelected(false);
                   cb3.setSelected(false);
                   cb4.setSelected(false);
                   cb5.setSelected(false);
                   cb6.setSelected(false);
                   cb7.setSelected(false);
                   cb8.setSelected(false);
           }
                          
   }
   
   
   
   
   public void checkBoxJob(HashMap<String,String> corrMuscles, JCheckBox cb) {
           
           if (cb.isSelected()) {
                   disableCorrMuscles(corrMuscles);
                   //loadProbes("adapted11_l.art");
                   System.out.print("-slected");
                  
           } 
                     
   }

   
   
   public void enableCorrMuscles(HashMap<String,String> corrMucle) {
                   
      for (Muscle muscle : myJawModel.myAttachedMuscles){
         muscle.setExcitationColor (Color.RED);
         muscle.setMaxColoredExcitation (1);
         myJawModel.addAxialSpring (muscle);
        
      }
       
   }
   
   
   
   public void disableCorrMuscles(HashMap<String,String> corrMucle) {
           
           for (String name: corrMucle.keySet()) {
                 AxialSpring as = myJawModel.axialSprings().get (name);
                 myJawModel.removeAxialSpring(as);      
           }
           
   }

    

   public void addClosingForce() throws IOException{      
      for (BodyConnector p : myJawModel.bodyConnectors ()){     
         if (p.getName ().equals ("BiteICP")==false){
            p.setEnabled (false);
            p.getRenderProps ().setVisible (false);
         }  
   }
      ((PlanarConnector)myJawModel.bodyConnectors ().get ("BiteICP")).setUnilateral (false);
      MuscleExciter mex=myJawModel.getMuscleExciters ().get ("bi_close");     
      NumericInputProbe probe = new NumericInputProbe (mex, "excitation",ArtisynthPath.getSrcRelativePath (JawModelFEM.class, "/data/input_activation.txt"));
      probe.setStartStopTimes (0, 1);
      probe.setName ("Closing Muscle Activation");
      addInputProbe (probe);
   }
   
   
   
   public void addOpening() throws IOException{
      for (BodyConnector p : myJawModel.bodyConnectors ()){         
            if (p.getName ().equals ("BiteICP")==false){
               p.setEnabled (false);
               p.getRenderProps ().setVisible (false);
            }        
      }           
      MuscleExciter mex=myJawModel.getMuscleExciters ().get ("bi_open");      
      NumericInputProbe probe = new NumericInputProbe (mex, "excitation",ArtisynthPath.getSrcRelativePath (JawModelFEM.class, "/data/input_activation.txt"));
      probe.setStartStopTimes (0, 0.5);
      probe.setName ("Opening Muscle Activation");
      addInputProbe (probe);
   }
      
   public void setWorkingDir() {
      if (workingDirname == null) return;
      // set default working directory to repository location
      File workingDir = new File (
      ArtisynthPath.getSrcRelativePath(JawFemDemoOptimizeSBME.class, workingDirname));
      ArtisynthPath.setWorkingDir(workingDir);        
   }
  
   public void loadProbes(String probesFilename) {
      String probeFileFullPath = (ArtisynthPath.getSrcRelativePath(JawModelFEM.class,"data/"+probesFilename));

      System.out.println("Loading Probes from File: " + probeFileFullPath);
       
      try {
          scanProbes(ArtisynthIO.newReaderTokenizer(probeFileFullPath));
       } catch (Exception e) {
          System.out.println("Error reading probe file");
          e.printStackTrace();
       }
    }
   
   
  public void addControlPanel(){
  
     ControlPanel panel;
     panel = new ControlPanel("Parameter Tuning","LiveUpdate");
     panel.addLabel ("Ligaments");
     panel.addWidget (myJawModel, "StmSlack");
     panel.addWidget (myJawModel, "SphmSlack");
     panel.addWidget ("tm_R", this, "models/jawmodel/multiPointSprings/tm_R:restLength");
     panel.addWidget ("tm_L", this, "models/jawmodel/multiPointSprings/tm_L:restLength");
     panel.addWidget (new JSeparator());
     panel.addLabel ("Elastic Foundation Contact");
     panel.addWidget (myJawModel, "EFYoung");
     panel.addWidget (myJawModel, "EFThickness");
     panel.addWidget (myJawModel, "EFDamping");
     panel.addWidget (myJawModel, "EFNu");
     panel.addWidget (new JSeparator());
     panel.addLabel ("Capsule Render Properties");
     panel.addWidget ("sapsule_r", this, "models/jawmodel/models/capsule_r:renderProps.visible");
     panel.addWidget ("sapsule_l", this, "models/jawmodel/models/capsule_l:renderProps.visible");
     addControlPanel (panel);
     panel.pack ();
     
    
  }
 
}
