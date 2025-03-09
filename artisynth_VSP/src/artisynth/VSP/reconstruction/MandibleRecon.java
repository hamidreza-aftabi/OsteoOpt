package artisynth.VSP.reconstruction;

import java.awt.Color;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.NumberFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JMenu;

import org.python.apache.xerces.impl.xpath.XPath.Step;

import artisynth.VSP.reconstruction.ReconAppRoot.DonorType;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.AxialSpringList;
import artisynth.core.modelbase.ComponentChangeEvent;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.modelbase.ScanWriteUtils;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.core.util.ArtisynthIO;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.ScanToken;
import artisynth.core.workspace.RootModel;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.LinearSpline3d;
import maspack.matrix.*;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.properties.PropertyUtils;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.PointStyle;
import maspack.util.IndentingPrintWriter;
import maspack.util.PathFinder;
import maspack.util.ReaderTokenizer;
import maspack.util.Write;
import maspack.widgets.GuiUtils;

/**
 * ArtiSynth root model that supports the creation of reconstruction
 * objects, such as cutting guides and meshes.
 *
 * The model itself contains a MechModel with customized containers for objects
 * related to the mandible and donor meshes, resection planes, the plate, donor
 * segments, cutting guides, and screw placement.  The MechModel also contains
 * a number of specialized worker components for creating and managing these
 * objects.
 */
public class MandibleRecon extends ReconAppRoot {

   TaskFrame myTaskFrame; // main GUI winow

   // worker classes:
   SegmentGenerator mySegmentGenerator;
   ImplantsManager myImplantsManager;
   DicomManager myDicomManager;
   DonorGuideBuilder myDonorGuideBuilder;
   MandibleGuideBuilder myMandibleGuideBuilder;
   PlateBuilder myPlateBuilder;

   // special containers:
   RenderableComponentList<FixedMeshBody> myResectionPlanes;
   PointList<FrameMarker> myPlateMarkers;
   PointList<FrameMarker> myAngleMarkers;
   PointList<FrameMarker> myDonorMarkers;
   PointList<FrameMarker> myDeformMarkers;
   RenderableComponentList<Implant> myImplants;
   RenderableComponentList<RigidBody> myRDPLineFrames;
   PointList<FrameMarker> myRDPLinePoints;
   AxialSpringList<AxialSpring> myRDPLineSegments;
   PointList<Point> myMandibleHandlePoints;
   RenderableComponentList<DonorSegmentBody> myDonorSegments;   
   RenderableComponentList<FixedMeshBody> myScrews;
   RenderableComponentList<FixedMeshBody> myTrimPlanes;

   File myCaseFile;     // most recent case file, if any
   File myExportFolder; // default folder for exporting reconstruction objects

   // properties definitions and accessors:

   public static PropertyList myProps =
      new PropertyList (MandibleRecon.class, ReconAppRoot.class);

   static {
      // nothing to add yet
   }
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   @Override
   public void setDonorSetBack (double d) {
      if (d != myDonorSetBack) {
         mySegmentGenerator.adjustDonorSetback (d - myDonorSetBack);
         super.setDonorSetBack (d);
      }
   }

   // other accessors:

   public File getExportFolder() {
      if (myExportFolder == null) {
         myExportFolder = getWorkingFolder();
      }
      return myExportFolder;
   }

   public void setExportFolder (File file) {
      myExportFolder = file;
   }
   
   public TaskFrame getTaskFrame() {
      return myTaskFrame;
   }

   public File getCaseFile() {
      return myCaseFile;
   }

   public void setCaseFile (File file) {
      myCaseFile = file;
   }

   public DonorGuideBuilder getDonorGuideBuilder() {
      return myDonorGuideBuilder;
   }

   public SegmentGenerator getSegmentGenerator() {
      return mySegmentGenerator;
   }

   public ImplantsManager getImplantsManager() {
      return myImplantsManager;
   }

   public DicomManager getDicomManager() {
      return myDicomManager;
   }

   public MandibleGuideBuilder getMandibleGuideBuilder() {
      return myMandibleGuideBuilder;
   }

   public PlateBuilder getPlateBuilder() {
      return myPlateBuilder;
   }
   
   // panel accessors

   public MeshesPanel getMeshesPanel() {
      return getTaskFrame().myMeshesPanel;
   }
         
   public DicomPanel getDicomPanel() {
      return getTaskFrame().myDicomPanel;
   }
         
   public ImplantsPanel getImplantsPanel() {
      return getTaskFrame().myImplantsPanel;
   }
         
   public SegmentsPanel getSegmentsPanel() {
      return getTaskFrame().mySegmentsPanel;
   }
         
   public DonorGuidePanel getDonorGuidePanel() {
      return getTaskFrame().myDonorGuidePanel;
   }
         
   public MandibleGuidePanel getMandibleGuidePanel() {
      return getTaskFrame().myMandibleGuidePanel;
   }

   public PlatePanel getPlatePanel() {
      return getTaskFrame().myPlatePanel;
   }

   /**
    * Root model build method. Creates the MechModel and populates it with
    * worker components and containers for reconstruction objects.
    */
   public void build (String[] args) throws IOException {

      // parse arguments
      File caseFile = null;
      File workingDir = null;
      boolean fibulaTest = false;
      boolean scapulaTest = false;
      boolean implantTest = false;
      boolean mandibleGuideTest = false;
      boolean xtest = false;
      boolean scapulaDonor = false;
      for (int i=0;i<args.length; i++) {
         if (args[i].equals ("-case")) {
            if (i == args.length-1) {
               System.out.println (
                  "WARNING: option -case requires another argument");
            }
            caseFile = new File(args[++i]);
         }
         else if (args[i].equals ("-workingDir")) {
            if (i == args.length-1) {
               System.out.println (
                  "WARNING: option -workingDir requires another argument");
            }
            workingDir = new File(args[++i]);
         }
         else if (args[i].equals ("-fibulaTest")) {
            fibulaTest = true;
         }
         else if (args[i].equals ("-xtest")) {
            xtest = true;
         }
         else if (args[i].equals ("-implantTest")) {
            implantTest = true;
         }
         else if (args[i].equals ("-scapulaTest")) {
            scapulaTest = true;
         }
         else if (args[i].equals ("-mandibleGuideTest")) {
            mandibleGuideTest = true;
         }
         else if (args[i].equals ("-scapulaDonor")) {
            scapulaDonor = true;
         }
         else {
            System.out.println ("WARNING: ignoring unknown option "+args[i]);
            System.out.println ("Options are:");
            System.out.println (" -case <fileName>  // load case from file");
            System.out.println (" -scapulaDonor     // donor will be scapula");
            System.out.println (" -fibulaTest       // create fibula test case");
            System.out.println (" -scapulaTest      // create scapula test case");
            System.out.println (" -implantTest      // create implant test case");
            System.out.println (" -mandibleGuideTest // mandible guide test case");
            System.out.println (" -workingDir <dir> // set working folder");
         }
      }

      myMech = new MechModel ("Reconstruction");
      // use static intergration for simulation
      myMech.setIntegrator (Integrator.StaticIncrementalStep);
      myMech.setGravity (0, 0, 0); // turn off gravity
      // default rendering properties for the MechModel:
      RenderProps.setPointStyle (myMech, PointStyle.SPHERE);
      RenderProps.setPointRadius (myMech, 1.25);
      RenderProps.setFaceColor (myMech, BONE_COLOR);
      RenderProps.setSpindleLines (myMech.axialSprings(), 1.25, Color.WHITE);
      addModel (myMech);

      // special containers for reconstruction objects
      myPlateMarkers =
         new PointList<FrameMarker> (FrameMarker.class, "plateMarkers");
      myMech.addFixed (myPlateMarkers);
      RenderProps.setSphericalPoints (myPlateMarkers, 1.25, MARKER_COLOR);
      myAngleMarkers =
         new PointList<FrameMarker> (FrameMarker.class, "angleMarkers");
      myMech.addFixed (myAngleMarkers);
      RenderProps.setSphericalPoints (myAngleMarkers, 1.25, LANDMARK_COLOR);
      myDonorMarkers =
         new PointList<FrameMarker> (FrameMarker.class, "donorMarkers");
      myMech.addFixed (myDonorMarkers);
      RenderProps.setSphericalPoints (myDonorMarkers, 1.25, MARKER_COLOR);
      myDeformMarkers =
         new PointList<FrameMarker> (FrameMarker.class, "deformMarkers");
      RenderProps.setVisible (myDeformMarkers, false);
      myMech.addFixed (myDeformMarkers);

      myImplants =
         new RenderableComponentList<Implant> (
            Implant.class, "implants", null);
      myMech.addFixed (myImplants);
      RenderProps.setFaceColor (myImplants, Color.LIGHT_GRAY);

      myRDPLineFrames =
         new RenderableComponentList<RigidBody> (
            RigidBody.class, "RDPLineFrames", null);
      myMech.addFixed (myRDPLineFrames);
      RenderProps.setFaceColor (myRDPLineFrames, Color.CYAN);

      myRDPLinePoints =
         new PointList<FrameMarker> (FrameMarker.class, "RDPLinePoints", null);
      myMech.addFixed (myRDPLinePoints);
      RenderProps.setVisible (myRDPLinePoints, false);

      myRDPLineSegments =
         new AxialSpringList<AxialSpring> (
            AxialSpring.class, "RDPLineSegments", null);
      myMech.addFixed (myRDPLineSegments);
      RenderProps.setCylindricalLines (myRDPLineSegments, 1.0, Color.CYAN);
 
      myMandibleHandlePoints =
         new PointList<Point> (
            Point.class, "mandibleHandlePoints", null);
      myMech.addFixed (myMandibleHandlePoints);
      RenderProps.setSphericalPoints (
         myMandibleHandlePoints,
         MandibleGuideBuilder.DEFAULT_HANDLE_RADIUS, Color.CYAN);

      myDonorSegments = 
         new RenderableComponentList<DonorSegmentBody> (
            DonorSegmentBody.class, "donorSegments");
      myMech.addFixed (myDonorSegments);
      RenderProps.setFaceColor (myDonorSegments, PLANE_COLOR);
      RenderProps.setBackColor (myDonorSegments, PLANE_BACK_COLOR);
      RenderProps.setFaceStyle (myDonorSegments, FaceStyle.FRONT_AND_BACK);

      myResectionPlanes = 
         new RenderableComponentList<FixedMeshBody> (
            FixedMeshBody.class, "resectionPlanes");
      myMech.addFixed (myResectionPlanes);
      RenderProps.setFaceColor (myResectionPlanes, PLANE_COLOR);

      myTrimPlanes = 
         new RenderableComponentList<FixedMeshBody> (
            FixedMeshBody.class, "trimPlanes");
      myMech.addFixed (myTrimPlanes);
      RenderProps.setFaceColor (myTrimPlanes, PLANE_COLOR);

      myScrews = 
         new RenderableComponentList<FixedMeshBody> (
            FixedMeshBody.class, "screws");
      myMech.addFixed (myScrews);
      RenderProps.setFaceColor (myScrews, PLATE_COLOR);

      // create worker componets:

      myMeshManager = new MeshManager ("meshManager");
      myMech.addFixed (myMeshManager);
      myMeshManager.initialize();

      mySegmentGenerator = new SegmentGenerator ("segmentGenerator");
      myMech.addFixed (mySegmentGenerator);
      mySegmentGenerator.initialize();

      myImplantsManager = new ImplantsManager ("implantsManager");
      myMech.addFixed (myImplantsManager);
      myImplantsManager.initialize();

      myDicomManager = new DicomManager ("dicomManager");
      myMech.addFixed (myDicomManager);
      myDicomManager.initialize();

      myDonorGuideBuilder = new DonorGuideBuilder ("donorGuideBuilder");
      myMech.addFixed (myDonorGuideBuilder);
      myDonorGuideBuilder.initialize();

      myMandibleGuideBuilder = new MandibleGuideBuilder ("mandibleGuideBuilder");
      myMech.addFixed (myMandibleGuideBuilder);
      myMandibleGuideBuilder.initialize();

      myPlateBuilder = new PlateBuilder ("plateBuilder");
      myMech.addFixed (myPlateBuilder);
      myPlateBuilder.initialize();

      if (scapulaDonor || scapulaTest) {
         setDonorType (DonorType.SCAPULA);
      }

      // create and place main GUI window:

      myTaskFrame = new TaskFrame ("Mandible reconstruction", this);
      Main.getMain().registerWindow (myTaskFrame);
      myTaskFrame.setVisible(true);
      JFrame frame = getMainFrame();
      if (frame != null) {
         java.awt.Point loc = frame.getLocation();
         myTaskFrame.setLocation (loc.x + frame.getWidth(), loc.y);
      }

      if (workingDir != null) {
         setWorkingFolder (workingDir);
      }
      else {
         setWorkingFolder (ArtisynthPath.getUserHomeFolder());
      }

      // handle options:
      if (caseFile != null) {
         // load in a case from a file
         loadCase (caseFile);
         setCaseFile (caseFile);
      }
      else if (fibulaTest) {
         // create a fibula test case
         createFibulaTestCase();
      }
      else if (mandibleGuideTest) {
         // create a mandible guid3 test case
         createMandibleGuideTestCase();
      }
      else if (scapulaTest) {
         // create a scapula test case
         createScapulaTestCase0();
      }
      else if (implantTest) {
         // create an implant test case
         createImplantTestCase();
      }
      else if (xtest) {
         // create an implant test case
         createXtestCase();
      }
   }

   protected boolean hasSpecialMenu() {
      return false;
   }

   /**
    * Dynamically create special menu.
    */
   protected void createSpecialMenu (JMenu menu) {
   }

   private void addImplant (RigidTransform3d TIW) {
      RigidBody implant = myImplantsManager.createImplant();
      implant.setPose (TIW);
      myMeshManager.addImplant (implant);
   }

   
   
   
   
   public void importFibulaOptimizationTwo () throws IOException {
      
      File fileDir = new File (
         PathFinder.getSourceRelativePath (this, "optimizationRBTest"));
      setWorkingFolder (fileDir);
      myTaskFrame.importMandible (new File (fileDir, "mandible_with_cartilage.obj"));
      myTaskFrame.importDonor (new File (fileDir, "fibula.obj"), true);
      myTaskFrame.importResectionPlanes (
         new File (fileDir, "resection_plane.txt"),
         MeshManager.PlaneFormat.ARTISYNTH);
      myTaskFrame.importMandibleMarkers (
         new File (fileDir, "mandible_markers.txt"),
         MeshManager.MarkerFormat.ARTISYNTH, true); 
      
   }

   

   
   
 public void createFibulaOptimizationTwo(double zOffset, double RDPOffset) throws IOException {
      
   
     
      RigidBody mandible = myMeshManager.getRigidBody("mandible");
      MeshBase mandibleMesh = mandible.getSurfaceMesh();
      
      int totalMarkers = 7; // Assuming you want the offset to reduce over 9 markers
      int markerIndex = 0;  // Initialize a counter for the marker index
      
      for (FrameMarker mkr : myPlateBuilder.getPlateMarkers())
      {
      
         double reductionFactor = Math.exp(-markerIndex / (double) totalMarkers);  // Exponential decay
         double zOffsetReduced = zOffset * reductionFactor;  // Apply the reduction to zOffset

         Point3d closestVertex = findClosestVertex(mandibleMesh, new Point3d (mkr.getPosition ().x, mkr.getPosition ().y, mkr.getPosition ().z+zOffsetReduced));
         mkr.setPosition (closestVertex);
         System.out.println("Closest Vertex to the Arbitrary Point: " + closestVertex);
         
       
         markerIndex++;  // Increment the marker index

         //Point3d loc = new Point3d();
         //loc.inverseTransform (mandible.getPose(), mkr.getPosition());
         //loc.z= loc.z+zOffset;
         //mkr.setLocation (loc);
         
         
      }
    
      
      myTaskFrame.mySegmentsPanel.createPlateCurve();
      myTaskFrame.mySegmentsPanel.createRDPLine();
      
    
      
      // creating parameter for RDP offset
      
      PointList<FrameMarker> rdpMarkers = (PointList<FrameMarker>)myMech.get("RDPLinePoints");
    
      FrameMarker middleRDPPoint = rdpMarkers.get (1);
      
      
      PolylineMesh plateCurveMesh = (PolylineMesh)myPlateBuilder.getPlateCurve().getMesh();
      LinearSpline3d plateCurve = new LinearSpline3d();
      
      ArrayList<Point3d> platePoints = new ArrayList<>();

      
      for (Vertex3d vtx : plateCurveMesh.getVertices()) {
         platePoints.add (vtx.getPosition());
      }
      plateCurve.setUsingDistance (platePoints);
      double plateLen = plateCurve.getSLength();
      
      
      // Calculate the arc length from the start of the spline to the middleRDPPoint
      double arcLength = getArcLengthToPoint(platePoints, middleRDPPoint.getPosition());

      System.out.println("Arc length to middleRDPPoint: " + arcLength);
      System.out.println("Total Plate Lenght" + plateLen);

      Vector3d newRDPvector = plateCurve.eval (arcLength + RDPOffset);
      Point3d newRDPposition = new  Point3d (newRDPvector.x, newRDPvector.y, newRDPvector.z);
      
      rdpMarkers.get (1).setPosition (newRDPposition);
      
      RigidBody rdpframe = (RigidBody)findComponent ("models/Reconstruction/RDPLineFrames/1");
      rdpframe.setPosition (newRDPposition);
      
      
      
      
      myTaskFrame.myMeshesPanel.clipMandible();
      myTaskFrame.myMeshesPanel.clipDonor();
      myMeshManager.addDonorMarker (new Point3d (-9.9232046, -2.575484, 88));
      myTaskFrame.mySegmentsPanel.findDonorSegments();

      
      String meshBodyPath1 = "models/Reconstruction/donorSegments/0";
      DonorSegmentBody meshBody1 = (DonorSegmentBody) findComponent(meshBodyPath1);

      String meshBodyPath2 = "models/Reconstruction/donorSegments/1";
      DonorSegmentBody meshBody2 = (DonorSegmentBody) findComponent(meshBodyPath2);

      RigidTransform3d originalPose1 = new RigidTransform3d(meshBody1.getPose());
      RigidTransform3d originalPose2 = new RigidTransform3d(meshBody2.getPose());

      double maxSum = Double.NEGATIVE_INFINITY;
      double optimalAngle1 = 0;
      double optimalAngle2 = 0;

      for (double angleDegrees1 = 10; angleDegrees1 < 40; angleDegrees1 += 1) { // Adjust step size as needed
          for (double angleDegrees2 = 10; angleDegrees2 < 40; angleDegrees2 += 1) { // Second loop for brute force search
              // Create the local rotation transform for the first donor segment
              RigidTransform3d localRotation1 = new RigidTransform3d();
              double angleRadians1 = Math.toRadians(angleDegrees1);
              localRotation1.setRotation(new AxisAngle(0, 0, 1, angleRadians1)); // Rotate around Z-axis

              // Create the local rotation transform for the second donor segment
              RigidTransform3d localRotation2 = new RigidTransform3d();
              double angleRadians2 = Math.toRadians(angleDegrees2);
              localRotation2.setRotation(new AxisAngle(0, 0, 1, angleRadians2)); // Rotate around Z-axis

              // Apply the local rotations to the original poses
              RigidTransform3d newPose1 = new RigidTransform3d();
              newPose1.mul(originalPose1, localRotation1);

              RigidTransform3d newPose2 = new RigidTransform3d();
              newPose2.mul(originalPose2, localRotation2);

              // Apply the new poses to the mesh bodies
              meshBody1.setPose(newPose1);
              meshBody2.setPose(newPose2);

              // Compute bony contact
              double[] ratio = myTaskFrame.computeBonyContact();

              // Sum the bony contact ratios
              double currentSum =  ratio[0] + ratio[1] + ratio[2] + ratio[3] + ratio[4] + ratio[5];
              if (currentSum > maxSum) {
                  maxSum = currentSum;
                  optimalAngle1 = angleDegrees1;
                  optimalAngle2 = angleDegrees2;
              }

              // Reset to the original poses
              meshBody1.setPose(originalPose1);
              meshBody2.setPose(originalPose2);
          }
          
          // Reset to the original poses
          meshBody1.setPose(originalPose1);
          meshBody2.setPose(originalPose2);
      }

      // Print or use the optimal angles found
      System.out.println("Optimal Angles for Donor Segment 0: " + optimalAngle1);
      System.out.println("Optimal Angles for Donor Segment 1: " + optimalAngle2);

      
      meshBody1.setPose(originalPose1);
      RigidTransform3d optimalRotation1 = new RigidTransform3d();
      optimalRotation1.setRotation(new AxisAngle(0, 0, 1, Math.toRadians(optimalAngle1)));
      RigidTransform3d optimalPose1 = new RigidTransform3d();
      optimalPose1.mul(originalPose1, optimalRotation1);
      meshBody1.setPose(optimalPose1);
      
      
      meshBody2.setPose(originalPose2);
      RigidTransform3d optimalRotation2 = new RigidTransform3d();
      optimalRotation2.setRotation(new AxisAngle(0, 0, 1, Math.toRadians(optimalAngle2)));
      RigidTransform3d optimalPose2 = new RigidTransform3d();
      optimalPose2.mul(originalPose2, optimalRotation2);
      meshBody2.setPose(optimalPose2);
      
    
      Point3d mesh2_position_initial = new Point3d(meshBody2.getPosition());
      Point3d mesh2_position = new Point3d(mesh2_position_initial);

      double maxShiftSum = 0;
      double optZShift = 0;
      double currentShiftSum = 0;
      double[] ratio;
      
      
      for (double zShift = -2; zShift < 2; zShift += 0.1) {
        
         // Reset mesh2_position to the initial position at the start of each loop iteration
         mesh2_position.set(mesh2_position_initial);

         // Apply the zShift
         mesh2_position.z -= zShift;
         meshBody2.setPosition(mesh2_position);
         
         ratio = myTaskFrame.computeBonyContact();

         // Sum the bony contact ratios
         currentShiftSum = ratio[2] + ratio[3];
         
         if (currentShiftSum > maxShiftSum) {
             maxShiftSum = currentShiftSum;
             optZShift = zShift;
         }
         
     }

     

      System.out.println("Optimal zshift for Donor Segment 1: " + optZShift);

      
      mesh2_position_initial.z = mesh2_position_initial.z  - optZShift;
      meshBody2.setPosition (mesh2_position_initial);
      
  

      myTaskFrame.mySegmentsPanel.createReconstruction ();
      myTaskFrame.myPlatePanel.createPlateFem ();
     
    
   }
   
     
   
   
 
 
 /**
  * Calculates the arc length from the start of the spline to a specific point.
  */
 public static double getArcLengthToPoint(ArrayList<Point3d> points, Point3d specificPoint) {
    double totalLength = 0.0;

    for (int i = 0; i < points.size() - 1; i++) {
       Point3d p0 = points.get(i);
       Point3d p1 = points.get(i + 1);

       // Check if the specific point lies within the current segment
       if (isPointInSegment(p0, p1, specificPoint)) {
          // Calculate the distance from p0 to the specific point
          totalLength += p0.distance(specificPoint);
          break;
       } else {
          // Add the full length of the current segment
          totalLength += p0.distance(p1);
       }
    }

    return totalLength;
 }

 /**
  * Checks if a specific point lies within a segment defined by two points.
  */
 public static boolean isPointInSegment(Point3d p0, Point3d p1, Point3d point) {
    double minX = Math.min(p0.x, p1.x);
    double maxX = Math.max(p0.x, p1.x);
    double minY = Math.min(p0.y, p1.y);
    double maxY = Math.max(p0.y, p1.y);
    double minZ = Math.min(p0.z, p1.z);
    double maxZ = Math.max(p0.z, p1.z);

    return (point.x >= minX && point.x <= maxX) &&
           (point.y >= minY && point.y <= maxY) &&
           (point.z >= minZ && point.z <= maxZ);
 }
   
    

  
  
  public void exportFemPlateTwo () {
     
     
     File exportFileDir = new File (
        PathFinder.getSourceRelativePath (this, "optimizationResultTwo"));
     setWorkingFolder (exportFileDir);
     
  
     FemModel3d plateFem = (FemModel3d)findComponent("models/Reconstruction/models/plateFem");
     
     //Vector3d translation = new Vector3d(7.0, -40.957,  -53.909);
     //plateFem.transformGeometry (new RigidTransform3d(translation, AxisAngle.IDENTITY));
     
    

     File artFile = new File(exportFileDir, "plate_opt.art");   
     
     maspack.util.NumberFormat fmt = new maspack.util.NumberFormat ("%7.7f\n");
     
     try (PrintWriter writer = new PrintWriter(new FileWriter(artFile))) {
        writer.println("FemModel3d");
        plateFem.write(writer, fmt, plateFem);
        System.out.println("Plate .art Model saved as " + artFile.getAbsolutePath());
    } catch (IOException e) {
        e.printStackTrace();
    }
    
     
  }
  
  
  
  public void exportFilesTwo () {
     
     File exportFileDir = new File (
        PathFinder.getSourceRelativePath (this, "optimizationResultTwo"));
     setWorkingFolder (exportFileDir);
     
     
     // mandible
     RigidBody mandibleL = myMeshManager.getRigidBody("mandibleL");
     RigidBody mandibleR = myMeshManager.getRigidBody("mandibleR");
     PolygonalMesh mesh_mandibleL = mandibleL.getSurfaceMesh().clone();
     PolygonalMesh mesh_mandibleR = mandibleR.getSurfaceMesh().clone();

     
     Vector3d translation = new Vector3d(7.0, -40.957,  -53.909);

     mesh_mandibleR.transform(new RigidTransform3d(translation, AxisAngle.IDENTITY));
     mesh_mandibleL.transform(new RigidTransform3d(translation, AxisAngle.IDENTITY));

    
     // mandible export      
     File exportFile_meshL = new File(exportFileDir, "resected_mandible_l_opt.obj");   
     File exportFile_meshR = new File(exportFileDir, "resected_mandible_r_opt.obj");   

     
     try {
        mesh_mandibleL.write(exportFile_meshL);
        System.out.println("Left Mandible Mesh exported successfully to: " + exportFile_meshL.getAbsolutePath());
    } catch (IOException e) {
        e.printStackTrace();
    }
     
     
     try {
        mesh_mandibleR.write(exportFile_meshR);
        System.out.println("Right Mandible Mesh exported successfully to: " + exportFile_meshR.getAbsolutePath());
    } catch (IOException e) {
        e.printStackTrace();
    }
     
     
     
     //screw     
     
     FixedMeshBody screw0 = (FixedMeshBody) findComponent("models/Reconstruction/screws/0");
     FixedMeshBody screw1 = (FixedMeshBody) findComponent("models/Reconstruction/screws/1");

     
     PolygonalMesh screwMesh0 = new PolygonalMesh();
     PolygonalMesh screwMesh0_export = new PolygonalMesh();

     PolygonalMesh screwMesh1 = new PolygonalMesh();
     PolygonalMesh screwMesh1_export = new PolygonalMesh();


     screwMesh0.addMesh((PolygonalMesh)screw0.getMesh(),/*respectTransform=*/true);
     screwMesh1.addMesh((PolygonalMesh)screw1.getMesh(),/*respectTransform=*/true);
     
     screwMesh0_export.addMesh((PolygonalMesh)screw0.getMesh(),/*respectTransform=*/true);
     screwMesh1_export.addMesh((PolygonalMesh)screw1.getMesh(),/*respectTransform=*/true);

     screwMesh0_export.transform(new RigidTransform3d(translation, AxisAngle.IDENTITY));
     screwMesh1_export.transform(new RigidTransform3d(translation, AxisAngle.IDENTITY));

    
    
     // screw export
     File exportFile_screw0 = new File(exportFileDir, "screw_opt0.obj");   
     
     try {
        screwMesh0_export.write(exportFile_screw0);
        System.out.println("screw0 Mesh exported successfully to: " + exportFile_screw0.getAbsolutePath());
    } catch (IOException e) {
        e.printStackTrace();
    }
     
     
     File exportFile_screw1 = new File(exportFileDir, "screw_opt1.obj");   
     
     try {
        screwMesh1_export.write(exportFile_screw1);
        System.out.println("screw1 Mesh exported successfully to: " + exportFile_screw1.getAbsolutePath());
    } catch (IOException e) {
        e.printStackTrace();
    }

     
     
     //donor
     PolygonalMesh mesh_donor0 = new PolygonalMesh();
     PolygonalMesh mesh_donor1 = new PolygonalMesh();

     
     RenderableComponentList<DonorSegmentBody> segmentList =  mySegmentGenerator.getSegments();
     
       
     mesh_donor0.addMesh (segmentList.get (0).getMesh(), /*respectTransforms=*/true);
     mesh_donor1.addMesh (segmentList.get (1).getMesh(), /*respectTransforms=*/true);

     
    
     removeDonorSetBack (mesh_donor0);
     removeDonorSetBack (mesh_donor1);

     
     SurfaceMeshIntersector intersector0 = new SurfaceMeshIntersector();
     PolygonalMesh intersector_mesh_donor0 = intersector0.findDifference01 (mesh_donor0, screwMesh0);
     intersector_mesh_donor0.transform(new RigidTransform3d(translation, AxisAngle.IDENTITY));
     
     SurfaceMeshIntersector intersector1 = new SurfaceMeshIntersector();
     
     
     PolygonalMesh intersector_mesh_donor1 = intersector1.findDifference01 (mesh_donor1, screwMesh1);
     intersector_mesh_donor1.transform(new RigidTransform3d(translation, AxisAngle.IDENTITY));

   
     // donor export
     File exportFile_donor0 = new File(exportFileDir, "donor_opt0.obj");   
     File exportFile_donor1 = new File(exportFileDir, "donor_opt1.obj");   

     
     try {
        intersector_mesh_donor0.write(exportFile_donor0);
        System.out.println("Donor0 Mesh exported successfully to: " + exportFile_donor0.getAbsolutePath());
    } catch (IOException e) {
        e.printStackTrace();
    }
     
     
     try {
        intersector_mesh_donor1.write(exportFile_donor1);
        System.out.println("Donor1 Mesh exported successfully to: " + exportFile_donor1.getAbsolutePath());
    } catch (IOException e) {
        e.printStackTrace();
    }
     
     
     
     //plate
     PolygonalMesh mesh_plate = myPlateBuilder.getPlateFemSurfaceWithHoles();
     mesh_plate.transform(new RigidTransform3d(translation, AxisAngle.IDENTITY));

     
     
     //plate export
     File exportFile_plate = new File(exportFileDir, "plate_opt.obj");   
     
     try {
        mesh_plate.write(exportFile_plate);
        System.out.println("Plate Mesh exported successfully to: " + exportFile_plate.getAbsolutePath());
    } catch (IOException e) {
        e.printStackTrace();
    }
    

        
  }
  
  
  
  
  public void importFibulaOptimizationS () throws IOException {
     
     
     File fileDir = new File (
        PathFinder.getSourceRelativePath (this, "optimizationSTest"));
     setWorkingFolder (fileDir);
     myTaskFrame.importMandible (new File (fileDir, "mandible_with_cartilage.obj"));
     myTaskFrame.importDonor (new File (fileDir, "fibula.obj"), true);
     myTaskFrame.importResectionPlanes (
        new File (fileDir, "resection_plane.txt"),
        MeshManager.PlaneFormat.ARTISYNTH);
     myTaskFrame.importMandibleMarkers (
        new File (fileDir, "mandible_markers.txt"),
        MeshManager.MarkerFormat.ARTISYNTH, true);

     
  }
    
  
  
 public void importFibulaOptimization () throws IOException {
    
    
    File fileDir = new File (
       PathFinder.getSourceRelativePath (this, "optimizationBTest"));
    setWorkingFolder (fileDir);
    myTaskFrame.importMandible (new File (fileDir, "mandible_with_cartilage.obj"));
    myTaskFrame.importDonor (new File (fileDir, "fibula.obj"), true);
    myTaskFrame.importResectionPlanes (
       new File (fileDir, "resection_plane.txt"),
       MeshManager.PlaneFormat.ARTISYNTH);
    myTaskFrame.importMandibleMarkers (
       new File (fileDir, "mandible_markers.txt"),
       MeshManager.MarkerFormat.ARTISYNTH, true);

    
 }
   
  
 
 public void createFibulaOptimization(double offset) throws IOException {
    
    double zOffset = offset;
    
  
   
    RigidBody mandible = myMeshManager.getRigidBody("mandible");
    MeshBase mandibleMesh = mandible.getSurfaceMesh();
    
  
    
    for (FrameMarker mkr : myPlateBuilder.getPlateMarkers())
    {
    
       Point3d closestVertex = findClosestVertex(mandibleMesh, new Point3d (mkr.getPosition ().x, mkr.getPosition ().y, mkr.getPosition ().z+zOffset));
       mkr.setPosition (closestVertex);
       System.out.println("Closest Vertex to the Arbitrary Point: " + closestVertex);
       
       //Point3d loc = new Point3d();
      //loc.inverseTransform (mandible.getPose(), mkr.getPosition());
       //loc.z= loc.z+zOffset;
       //mkr.setLocation (loc);
       
       
    }
  
    
    myTaskFrame.mySegmentsPanel.createPlateCurve();
    myTaskFrame.mySegmentsPanel.createRDPLine();
    myTaskFrame.myMeshesPanel.clipMandible();
    myTaskFrame.myMeshesPanel.clipDonor();
    myMeshManager.addDonorMarker (
       new Point3d (-9.9232046, -2.575484, 88));
    myTaskFrame.mySegmentsPanel.findDonorSegments();

    

    String meshBodyPath = "models/Reconstruction/donorSegments/0";
    DonorSegmentBody meshBody = (DonorSegmentBody) findComponent(meshBodyPath);


   
    RigidTransform3d originalPose = new RigidTransform3d(meshBody.getPose());

    double maxSum = Double.NEGATIVE_INFINITY;
    double optimalAngle = 0;

    for (double angleDegrees = -30; angleDegrees <90; angleDegrees += 1) { // Adjust step size as needed
      

        // Create the local rotation transform
        RigidTransform3d localRotation = new RigidTransform3d();
        double angleRadians = Math.toRadians(angleDegrees);
        localRotation.setRotation(new AxisAngle(0, 0, 1, angleRadians)); // Rotate around Z-axis

        // Apply the local rotation to the original pose
        RigidTransform3d newPose = new RigidTransform3d();
        newPose.mul(originalPose, localRotation);

        // Apply the new pose to the mesh body
        meshBody.setPose(newPose);

        // Compute bony contact
        double[] ratio = myTaskFrame.computeBonyContact();

        // Check if the current sum is greater than the maximum found so far
        double currentSum = ratio[1] + ratio[2];
        if (currentSum > maxSum) {
            maxSum = currentSum;
            optimalAngle = angleDegrees;
        }
        
        // Reset to the original pose
        meshBody.setPose(originalPose);
    }

    
    
    // Apply the optimal rotation found
    meshBody.setPose(originalPose);
    RigidTransform3d optimalRotation = new RigidTransform3d();
    optimalRotation.setRotation(new AxisAngle(0, 0, 1, Math.toRadians(optimalAngle)));
    RigidTransform3d optimalPose = new RigidTransform3d();
    optimalPose.mul(originalPose, optimalRotation);
    meshBody.setPose(optimalPose);

    System.out.println("Optimal rotation angle: " + optimalAngle + " degrees with max sum: " + maxSum);
    
    myTaskFrame.mySegmentsPanel.createReconstruction ();
    myTaskFrame.myPlatePanel.createPlateFem ();
 
    
    //myTaskFrame.myPlatePanel.createScrews ();
    

    /*      
    // Create the local rotation transform
    RigidTransform3d localRotation = new RigidTransform3d();
    double angleDegrees = 90; // Rotate 45 degrees as an example
    double angleRadians = Math.toRadians(angleDegrees);
    localRotation.setRotation(new AxisAngle(0, 0, 1, angleRadians)); // Rotate around Z-axis

    // Convert the local rotation to world coordinates
    AffineTransform3d worldTransform = new AffineTransform3d();
    worldTransform.mul(localTransform, localRotation);
    worldTransform.mulInverseRight(worldTransform, localTransform);

    // Apply the transformation to the mesh
    //mesh.transform(worldTransform);
    meshBody.transformGeometry (worldTransform);
    //mesh.transform (worldTransform);

    System.out.println("Successfully rotated the mesh around its local axis by " + angleDegrees + " degrees.");


    double[] ratio = myTaskFrame.computeBonyContact();
    
    System.out.println("Bony Contact" + ratio[1]+ ratio[2]);
    */
    

    
    /*
    double optimalAngle = findOptimalRotation(donorSegment, mandibleR);
    System.out.println("Optimal Rotation Angle (degrees): " + optimalAngle);
    
    System.out.println("Optimal Rotation Angle (degrees): " + optimalAngle);

    // Apply the optimal rotation to the donor segment
    rotateDonorSegment(donorSegment, optimalAngle);

    // Print the applied rotation for verification
    System.out.println("Applied Rotation: " + donorSegment.getPose());
    */
    
    
    
    /*
    myTaskFrame.mySegmentsPanel.createReconstruction();

    Point3d p0 = new Point3d (-17.118676, -27.972629, 2.2921666);
    Point3d p1 = new Point3d (0.18963657, -42.16102, -2.6552247);
    Point3d p2 = new Point3d (25.610483, -9.015806, 4.5762074);
    OcclusalPlane pbody = myImplantsManager.createOcclusalPlane (p1, p0, p2);

    myMeshManager.addOcclusalPlane (pbody);
    myTaskFrame.setOcclusalPlaneVisible (false);   


    // add implants
    RigidTransform3d TIW_0 = new RigidTransform3d (-41.1733, 5.91287, -4.66524);
    TIW_0.R.setAxisAngle (0.15442, 0.17364, -0.97263, DTOR*70.71);
    addImplant (TIW_0);

    RigidTransform3d TIW_1 = new RigidTransform3d (-33.0932, -7.44589,-8.78361);
    TIW_1.R.setAxisAngle (0.21129, 0.18573, -0.95962, DTOR*57.105);
    addImplant (TIW_1);

    RigidTransform3d TIW_2 = new RigidTransform3d (-26.737, -17.9732, -12.0284);
    TIW_2.R.setAxisAngle (0.24725, 0.19312, -0.94951, DTOR*50.755);
    addImplant (TIW_2);

    RigidTransform3d TIW_3 = new RigidTransform3d (-17.3797, -25.5598,-15.3652);
    TIW_3.R.setAxisAngle (0.29795, 0.20317, -0.93271, DTOR*43.783);
    addImplant (TIW_3);
    
    myTaskFrame.myPlatePanel.createPlateFem ();
    
   
    // mandible
    RigidBody mandibleL = myMeshManager.getRigidBody("mandibleL");
    RigidBody mandibleR = myMeshManager.getRigidBody("mandibleR");
    PolygonalMesh mesh = mandibleL.getSurfaceMesh().clone();
    mesh.addMesh (mandibleR.getSurfaceMesh(),true);
    
   
    // export
    File exportFileDir = new File (
       PathFinder.getSourceRelativePath (this, "optimizationresult"));
    setWorkingFolder (exportFileDir);
    
    File exportFile = new File(exportFileDir, "ResectedMandible.obj");   
    
    try {
       mesh.write(exportFile);
       System.out.println("Mesh exported successfully to: " + exportFile.getAbsolutePath());
   } catch (IOException e) {
       e.printStackTrace();
   }
   */
    
  
 }
 
 

   
 
 
 
 
 //**************** NOT USED ***************************
  
   public void createFibulaOptimizationS(double offset) throws IOException {
      
      double zOffset = offset;
      
    
     
      RigidBody mandible = myMeshManager.getRigidBody("mandible");
      MeshBase mandibleMesh = mandible.getSurfaceMesh();
      
    
      
      for (FrameMarker mkr : myPlateBuilder.getPlateMarkers())
      {
      
         Point3d closestVertex = findClosestVertex(mandibleMesh, new Point3d (mkr.getPosition ().x, mkr.getPosition ().y, mkr.getPosition ().z+zOffset));
         mkr.setPosition (closestVertex);
         System.out.println("Closest Vertex to the Arbitrary Point: " + closestVertex);
         
         
         //Point3d loc = new Point3d();
         //loc.inverseTransform (mandible.getPose(), mkr.getPosition());
         //loc.z= loc.z+zOffset;
         //mkr.setLocation (loc);
         
         
      }
      
      
      myTaskFrame.mySegmentsPanel.createPlateCurve();
      myTaskFrame.mySegmentsPanel.createRDPLine();
      myTaskFrame.myMeshesPanel.clipMandible();
      myTaskFrame.myMeshesPanel.clipDonor();
      myMeshManager.addDonorMarker (
         new Point3d (-9.9232046, -2.575484, 88));
      myTaskFrame.mySegmentsPanel.findDonorSegments();

      
 
      String meshBodyPath = "models/Reconstruction/donorSegments/0";
      DonorSegmentBody meshBody = (DonorSegmentBody) findComponent(meshBodyPath);


     
      RigidTransform3d originalPose = new RigidTransform3d(meshBody.getPose());

      double maxSum = Double.NEGATIVE_INFINITY;
      double optimalAngle = 0;
      double optYShift = 0;


      Point3d mesh_position_initial = new Point3d(meshBody.getPosition());
      Point3d mesh_position = new Point3d(mesh_position_initial);

 
  

      for (double angleDegrees = -10; angleDegrees <100; angleDegrees += 1) { // Adjust step size as needed
         for (double yShift = -1; yShift < 1; yShift += 0.1) {

          // Create the local rotation transform
          RigidTransform3d localRotation = new RigidTransform3d();
          double angleRadians = Math.toRadians(angleDegrees);
          localRotation.setRotation(new AxisAngle(0, 0, 1, angleRadians)); // Rotate around Z-axis

          // Apply the local rotation to the original pose
          RigidTransform3d newPose = new RigidTransform3d();
          newPose.mul(originalPose, localRotation);

          // Apply the new pose to the mesh body
          meshBody.setPose(newPose);
          
          
          mesh_position.y -= yShift;
          meshBody.setPosition(mesh_position);
          
          
          // Compute bony contact
          double[] ratio = myTaskFrame.computeBonyContact();

          // Check if the current sum is greater than the maximum found so far
          double currentSum = ratio[1] + ratio[2];
          if (currentSum > maxSum) {
              maxSum = currentSum;
              optimalAngle = angleDegrees;
              optYShift = yShift;
          }
          
          // Reset to the original pose
          meshBody.setPose(originalPose);
          mesh_position.set(mesh_position_initial);

         }
         meshBody.setPose(originalPose);
         mesh_position.set(mesh_position_initial);
      }

      
      
      // Apply the optimal rotation found
      meshBody.setPose(originalPose);
      RigidTransform3d optimalRotation = new RigidTransform3d();
      optimalRotation.setRotation(new AxisAngle(0, 0, 1, Math.toRadians(optimalAngle)));
      RigidTransform3d optimalPose = new RigidTransform3d();
      optimalPose.mul(originalPose, optimalRotation);
      meshBody.setPose(optimalPose);

      System.out.println("Optimal rotation angle: " + optimalAngle + " degrees with max sum: " + maxSum);
      
      
      mesh_position_initial.y = mesh_position_initial.y  - optYShift;
      meshBody.setPosition (mesh_position_initial);
      
      
      System.out.println("Optimal Yshift for Donor Segment 1: " + optYShift);

      
      myTaskFrame.mySegmentsPanel.createReconstruction ();
      myTaskFrame.myPlatePanel.createPlateFem ();
   

    
   }
   
   

     
   
   
   
   
   
   
   
   public void exportFemPlate () {
    
      
      File exportFileDir = new File (
         PathFinder.getSourceRelativePath (this, "optimizationResult"));
      setWorkingFolder (exportFileDir);
      
   
      FemModel3d plateFem = (FemModel3d)findComponent("models/Reconstruction/models/plateFem");
      
      //Vector3d translation = new Vector3d(7.0, -40.957,  -53.909);
      //plateFem.transformGeometry (new RigidTransform3d(translation, AxisAngle.IDENTITY));
      
     

      File artFile = new File(exportFileDir, "plate_opt.art");   
      
      maspack.util.NumberFormat fmt = new maspack.util.NumberFormat ("%7.7f\n");
      
      try (PrintWriter writer = new PrintWriter(new FileWriter(artFile))) {
         writer.println("FemModel3d");
         plateFem.write(writer, fmt, plateFem);
         System.out.println("Plate .art Model saved as " + artFile.getAbsolutePath());
     } catch (IOException e) {
         e.printStackTrace();
     }
     
      
   }
   
   
   
   
   
   
   
   

   
   public void exportFiles () {
      
      File exportFileDir = new File (
         PathFinder.getSourceRelativePath (this, "optimizationResult"));
      setWorkingFolder (exportFileDir);
      
      
      // mandible
      RigidBody mandibleL = myMeshManager.getRigidBody("mandibleL");
      RigidBody mandibleR = myMeshManager.getRigidBody("mandibleR");
      PolygonalMesh mesh_mandibleL = mandibleL.getSurfaceMesh().clone();
      PolygonalMesh mesh_mandibleR = mandibleR.getSurfaceMesh().clone();

      
      Vector3d translation = new Vector3d(7.0, -40.957,  -53.909);

      mesh_mandibleR.transform(new RigidTransform3d(translation, AxisAngle.IDENTITY));
      mesh_mandibleL.transform(new RigidTransform3d(translation, AxisAngle.IDENTITY));

     
      // mandible export      
      File exportFile_meshL = new File(exportFileDir, "resected_mandible_l_opt.obj");   
      File exportFile_meshR = new File(exportFileDir, "resected_mandible_r_opt.obj");   

      
      try {
         mesh_mandibleL.write(exportFile_meshL);
         System.out.println("Left Mandible Mesh exported successfully to: " + exportFile_meshL.getAbsolutePath());
     } catch (IOException e) {
         e.printStackTrace();
     }
      
      
      try {
         mesh_mandibleR.write(exportFile_meshR);
         System.out.println("Right Mandible Mesh exported successfully to: " + exportFile_meshR.getAbsolutePath());
     } catch (IOException e) {
         e.printStackTrace();
     }
      
      
      //screw
      PolygonalMesh mesh_screw0 = myPlateBuilder.getMandibleScrewMesh();
      PolygonalMesh mesh_screw = myPlateBuilder.getMandibleScrewMesh();
      mesh_screw.transform(new RigidTransform3d(translation, AxisAngle.IDENTITY));

      
      
      // screw export
      File exportFile_screw = new File(exportFileDir, "screw_opt0.obj");   
      
      try {
         mesh_screw.write(exportFile_screw);
         System.out.println("screw0 Mesh exported successfully to: " + exportFile_screw.getAbsolutePath());
     } catch (IOException e) {
         e.printStackTrace();
     }

      
      
      //donor
      PolygonalMesh mesh_donor = new PolygonalMesh();
      for (DonorSegmentBody seg : mySegmentGenerator.getSegments()) {
         if (getDonorType() == DonorType.SCAPULA) {
            PolygonalMesh meshInD = seg.getMesh().clone();
            meshInD.setMeshToWorld (seg.getPoseD());
            mesh_donor.addMesh (meshInD, /*respectTransforms=*/true);
         }
         else {
            mesh_donor.addMesh (seg.getMesh(), /*respectTransforms=*/true);
         }
      }
      removeDonorSetBack (mesh_donor);
      
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();
      PolygonalMesh intersector_mesh_donor = intersector.findDifference01 (mesh_donor, mesh_screw0);
      
      intersector_mesh_donor.transform(new RigidTransform3d(translation, AxisAngle.IDENTITY));

    
      // donor export
      File exportFile_donor = new File(exportFileDir, "donor_opt0.obj");   
      
      try {
         intersector_mesh_donor.write(exportFile_donor);
         System.out.println("Donor Mesh exported successfully to: " + exportFile_donor.getAbsolutePath());
     } catch (IOException e) {
         e.printStackTrace();
     }
      
      
      
      //plate
     
      PolygonalMesh mesh_plate = myPlateBuilder.getPlateFemSurfaceWithHoles();
      mesh_plate.transform(new RigidTransform3d(translation, AxisAngle.IDENTITY));

      
      
      //plate export
      File exportFile_plate = new File(exportFileDir, "plate_opt.obj");   
      
      try {
         mesh_plate.write(exportFile_plate);
         System.out.println("Plate Mesh exported successfully to: " + exportFile_plate.getAbsolutePath());
     } catch (IOException e) {
         e.printStackTrace();
     }
     

         
   }
   
   
   
   
   
   
   
   
   
   /**
    * Specialized method for creating a fibula test case using known input
    * files and the methods provided in TaskFrame.
    */
   public void createFibulaTestCase() throws IOException {
      
      double zOffset = 0;
      
      
      File fileDir = new File (
         PathFinder.getSourceRelativePath (this, "optimizationBTest"));
      setWorkingFolder (fileDir);
      myTaskFrame.importMandible (new File (fileDir, "mandible_with_cartilage.obj"));
      myTaskFrame.importDonor (new File (fileDir, "fibula.obj"), true);
      myTaskFrame.importResectionPlanes (
         new File (fileDir, "resection_plane.txt"),
         MeshManager.PlaneFormat.ARTISYNTH);
      myTaskFrame.importMandibleMarkers (
         new File (fileDir, "mandible_markers.txt"),
         MeshManager.MarkerFormat.ARTISYNTH, true);
      
      
      RigidBody mandible = myMeshManager.getRigidBody("mandible");
      MeshBase mandibleMesh = mandible.getSurfaceMesh();
      
    
      
      for (FrameMarker mkr : myPlateBuilder.getPlateMarkers())
      {
      
         Point3d closestVertex = findClosestVertex(mandibleMesh, new Point3d (mkr.getPosition ().x, mkr.getPosition ().y, mkr.getPosition ().z+zOffset));
         mkr.setPosition (closestVertex);
         //System.out.println("Closest Vertex to the Arbitrary Point: " + closestVertex);
         
      }
    
      
      myTaskFrame.mySegmentsPanel.createPlateCurve();
      myTaskFrame.mySegmentsPanel.createRDPLine();
      myTaskFrame.myMeshesPanel.clipMandible();
      myTaskFrame.myMeshesPanel.clipDonor();
      myMeshManager.addDonorMarker (
         new Point3d (-9.3495352, 0.029357566, 75.68918));
      myTaskFrame.mySegmentsPanel.findDonorSegments();

      
 
      String meshBodyPath = "models/Reconstruction/donorSegments/0";
      DonorSegmentBody meshBody = (DonorSegmentBody) findComponent(meshBodyPath);


     
      RigidTransform3d originalPose = new RigidTransform3d(meshBody.getPose());

      double maxSum = Double.NEGATIVE_INFINITY;
      double optimalAngle = 0;

      for (double angleDegrees = 0; angleDegrees < 360; angleDegrees += 1) { // Adjust step size as needed
        

          // Create the local rotation transform
          RigidTransform3d localRotation = new RigidTransform3d();
          double angleRadians = Math.toRadians(angleDegrees);
          localRotation.setRotation(new AxisAngle(0, 0, 1, angleRadians)); // Rotate around Z-axis

          // Apply the local rotation to the original pose
          RigidTransform3d newPose = new RigidTransform3d();
          newPose.mul(originalPose, localRotation);

          // Apply the new pose to the mesh body
          meshBody.setPose(newPose);

          // Compute bony contact
          double[] ratio = myTaskFrame.computeBonyContact();

          // Check if the current sum is greater than the maximum found so far
          double currentSum = ratio[1] + ratio[2];
          if (currentSum > maxSum) {
              maxSum = currentSum;
              optimalAngle = angleDegrees;
          }
          
          // Reset to the original pose
          meshBody.setPose(originalPose);
      }

      
      
      // Apply the optimal rotation found
      meshBody.setPose(originalPose);
      RigidTransform3d optimalRotation = new RigidTransform3d();
      optimalRotation.setRotation(new AxisAngle(0, 0, 1, Math.toRadians(optimalAngle)));
      RigidTransform3d optimalPose = new RigidTransform3d();
      optimalPose.mul(originalPose, optimalRotation);
      meshBody.setPose(optimalPose);

      System.out.println("Optimal rotation angle: " + optimalAngle + " degrees with max sum: " + maxSum);
      
      myTaskFrame.mySegmentsPanel.createReconstruction ();
      myTaskFrame.myPlatePanel.createPlateFem ();
   
      
      //myTaskFrame.myPlatePanel.createScrews ();
      
 
      /*      
      // Create the local rotation transform
      RigidTransform3d localRotation = new RigidTransform3d();
      double angleDegrees = 90; // Rotate 45 degrees as an example
      double angleRadians = Math.toRadians(angleDegrees);
      localRotation.setRotation(new AxisAngle(0, 0, 1, angleRadians)); // Rotate around Z-axis

      // Convert the local rotation to world coordinates
      AffineTransform3d worldTransform = new AffineTransform3d();
      worldTransform.mul(localTransform, localRotation);
      worldTransform.mulInverseRight(worldTransform, localTransform);

      // Apply the transformation to the mesh
      //mesh.transform(worldTransform);
      meshBody.transformGeometry (worldTransform);
      //mesh.transform (worldTransform);

      System.out.println("Successfully rotated the mesh around its local axis by " + angleDegrees + " degrees.");
 
 
      double[] ratio = myTaskFrame.computeBonyContact();
      
      System.out.println("Bony Contact" + ratio[1]+ ratio[2]);
      */
      

      
      /*
      double optimalAngle = findOptimalRotation(donorSegment, mandibleR);
      System.out.println("Optimal Rotation Angle (degrees): " + optimalAngle);
      
      System.out.println("Optimal Rotation Angle (degrees): " + optimalAngle);

      // Apply the optimal rotation to the donor segment
      rotateDonorSegment(donorSegment, optimalAngle);

      // Print the applied rotation for verification
      System.out.println("Applied Rotation: " + donorSegment.getPose());
      */
      
      
      
      /*
      myTaskFrame.mySegmentsPanel.createReconstruction();

      Point3d p0 = new Point3d (-17.118676, -27.972629, 2.2921666);
      Point3d p1 = new Point3d (0.18963657, -42.16102, -2.6552247);
      Point3d p2 = new Point3d (25.610483, -9.015806, 4.5762074);
      OcclusalPlane pbody = myImplantsManager.createOcclusalPlane (p1, p0, p2);

      myMeshManager.addOcclusalPlane (pbody);
      myTaskFrame.setOcclusalPlaneVisible (false);   


      // add implants
      RigidTransform3d TIW_0 = new RigidTransform3d (-41.1733, 5.91287, -4.66524);
      TIW_0.R.setAxisAngle (0.15442, 0.17364, -0.97263, DTOR*70.71);
      addImplant (TIW_0);

      RigidTransform3d TIW_1 = new RigidTransform3d (-33.0932, -7.44589,-8.78361);
      TIW_1.R.setAxisAngle (0.21129, 0.18573, -0.95962, DTOR*57.105);
      addImplant (TIW_1);

      RigidTransform3d TIW_2 = new RigidTransform3d (-26.737, -17.9732, -12.0284);
      TIW_2.R.setAxisAngle (0.24725, 0.19312, -0.94951, DTOR*50.755);
      addImplant (TIW_2);

      RigidTransform3d TIW_3 = new RigidTransform3d (-17.3797, -25.5598,-15.3652);
      TIW_3.R.setAxisAngle (0.29795, 0.20317, -0.93271, DTOR*43.783);
      addImplant (TIW_3);
      
      myTaskFrame.myPlatePanel.createPlateFem ();
      
     
      // mandible
      RigidBody mandibleL = myMeshManager.getRigidBody("mandibleL");
      RigidBody mandibleR = myMeshManager.getRigidBody("mandibleR");
      PolygonalMesh mesh = mandibleL.getSurfaceMesh().clone();
      mesh.addMesh (mandibleR.getSurfaceMesh(),true);
      
     
      // export
      File exportFileDir = new File (
         PathFinder.getSourceRelativePath (this, "optimizationresult"));
      setWorkingFolder (exportFileDir);
      
      File exportFile = new File(exportFileDir, "ResectedMandible.obj");   
      
      try {
         mesh.write(exportFile);
         System.out.println("Mesh exported successfully to: " + exportFile.getAbsolutePath());
     } catch (IOException e) {
         e.printStackTrace();
     }
     */
      
    
   }
   
  
private static boolean arePropertiesInitialized(ModelComponent component) {
   // Add checks to verify that all necessary properties of the component are initialized
   // For example, if the component is an instance of FemModel3d, you can check its properties:
   // return component instanceof FemModel3d && ((FemModel3d) component).getNodes() != null;

   // Placeholder check, update with actual necessary property checks
   return component != null;
}
   
      public static Point3d findClosestVertex(MeshBase mesh, Point3d arbitraryPoint) {
          Vertex3d closestVertex = null;
          double minDistance = 15;

          for (Vertex3d vertex : mesh.getVertices()) {
              double distance = vertex.getWorldPoint().distance(arbitraryPoint);

              if (distance < minDistance) {
                  minDistance = distance;
                  closestVertex = vertex;
              }
          }

          return closestVertex != null ? closestVertex.getWorldPoint() : null;
      }

   /**
    * Specialized method for creating a mandible guide test case.
    */
   private void createMandibleGuideTestCase() throws IOException {
      createFibulaTestCase();
      myTaskFrame.myMandibleGuidePanel.createCutBoxes();
      myTaskFrame.myMandibleGuidePanel.alignToMandible();
      myTaskFrame.myMandibleGuidePanel.addFlanges();
   }

   /**
    * Specialized method for creating a scapula test case using known input
    * files and the methods provided in TaskFrame.
    */
   private void createScapulaTestCase0() throws IOException {
      File fileDir = new File (
         PathFinder.getSourceRelativePath (this, "mandibleFibulaTest"));
      setWorkingFolder (fileDir);
      myTaskFrame.importMandible (new File (fileDir, "mandibleSimp0.stl"));
      myTaskFrame.importDonor (new File (fileDir, "leftScapulaSimp0.stl"), true);
      myTaskFrame.importResectionPlanes (
         new File (fileDir, "resectPlanes0.txt"),
         MeshManager.PlaneFormat.ARTISYNTH);
     
      myTaskFrame.importMandibleMarkers (
         new File (fileDir, "mandibleMarkers0.txt"),
         MeshManager.MarkerFormat.ARTISYNTH, true);
      myTaskFrame.mySegmentsPanel.createPlateCurve();
      mySegmentGenerator.setMaxSegments (1);
      myTaskFrame.mySegmentsPanel.createRDPLine();
      myTaskFrame.myMeshesPanel.clipMandible();

      myMeshManager.addDonorMarkerLoc (
         new Point3d (21.7119, -54.4531, 4.41422));
      myMeshManager.addDonorMarkerLoc (
         new Point3d (27.6425, 8.26332, -2.69181));
      //RenderProps.setVisible (myTrimPlanes, false);
      myMeshManager.clearAngleMarkers();
      setDefaultViewOrientation (AxisAlignedRotation.NY_Z);
      setDonorSetBack (90);
      RigidBody donor = myMeshManager.getRigidBody ("donor");
      RigidTransform3d TDW = new RigidTransform3d (donor.getPose());
      TDW.R.setAxisAngle (0.80838, 0.41625, 0.41625, DTOR*102.1);
      donor.setPose (TDW);
   }

   /**
    * Specialized method for creating a scapula test case using known input
    * files and the methods provided in TaskFrame.
    */
   private void createScapulaTestCase1() throws IOException {
      File fileDir = new File (
         PathFinder.getSourceRelativePath (this, "mandibleScapulaTest"));
      setWorkingFolder (fileDir);
      myTaskFrame.importMandible (new File (fileDir, "mandibleSimp1.stl"));
      myTaskFrame.importDonor (new File (fileDir, "scapulaSimp1.stl"), true);
      myTaskFrame.importResectionPlanes (
         new File (fileDir, "resectPlanes1.txt"),
         MeshManager.PlaneFormat.ARTISYNTH);
      myTaskFrame.importMandibleMarkers (
         new File (fileDir, "mandibleMarkers1.txt"),
         MeshManager.MarkerFormat.ARTISYNTH, true);
      myTaskFrame.mySegmentsPanel.createPlateCurve();
      myTaskFrame.mySegmentsPanel.createRDPLine();
      myTaskFrame.myMeshesPanel.clipMandible();
      myMeshManager.addDonorMarker (
         new Point3d (24.263652, 53.269805, -61.424176));
      myMeshManager.addDonorMarker (
         new Point3d (26.74235, 59.32085, -4.355212));
      //myTaskFrame.findDonorSegments();
      //myTaskFrame.createReconstruction();
      //RenderProps.setVisible (myTrimPlanes, false);
      myMeshManager.clearAngleMarkers();
      setDefaultViewOrientation (AxisAlignedRotation.Y_Z);
      setDonorSetBack (90);
      RigidBody donor = myMeshManager.getRigidBody ("donor");
      RigidTransform3d TDW = new RigidTransform3d (donor.getPose());
      TDW.R.setAxisAngle (0.80838, 0.41625, 0.41625, DTOR*102.1);
      donor.setPose (TDW);
   }

   /**
    * Specialized method for creating an implant test case using known input
    * files and the methods provided in TaskFrame.
    */
   private void createImplantTestCase() throws IOException {
      File fileDir = new File (
         System.getProperty ("user.home"), "tmp/mandibleMaxilla2");
      setWorkingFolder (fileDir);
      myTaskFrame.importMandible (new File (fileDir, "Mandible.stl"));
      myDicomManager.createViewer (new File (fileDir, "dicom"));
      myTaskFrame.importMaxilla (new File (fileDir, "Maxilla.stl"));
      Point3d p0 = new Point3d (-23.78793, -2.6442898, 8.0506397);
      Point3d p1 = new Point3d (-2.3220838, -41.294101, -3.0069555);
      Point3d p2 = new Point3d (24.511117, -4.3152973, 6.0998097);
      OcclusalPlane pbody = myImplantsManager.createOcclusalPlane (p1, p0, p2);
      myMeshManager.addOcclusalPlane (pbody);

      //myTaskFrame.createOcclusalSlice (pbody.getPose());
      //myTaskFrame.setOcclusalSliceVisible (false);

      rerender();
   }

   private void createXtestCase() throws IOException {
      File fileDir = new File (
         System.getProperty ("user.home"), "tmp/guidebug");
      setWorkingFolder (fileDir);
      myTaskFrame.importMandible (new File (fileDir, "MandibleRemesh.stl"));
      myTaskFrame.importResectionPlanes (
         new File (fileDir, "planes.txt"),
         MeshManager.PlaneFormat.ARTISYNTH);
      myTaskFrame.importMandibleMarkers (
         new File (fileDir, "fids2.txt"),
         MeshManager.MarkerFormat.ARTISYNTH, true);
      myTaskFrame.mySegmentsPanel.createPlateCurve();
      myTaskFrame.myMandibleGuidePanel.createCutBoxes();
      //myTaskFrame.myMandibleGuidePanel.alignToMandible();
   }

   private void writeFilePath (PrintWriter pw, String name, File file) {
      pw.println (name + Write.getQuotedString(file.toString()));
   }

   /**
    * Saves a case to a file, by writing out its MechModel and the
    * properties of the root model.
    */
   public void saveCase (File file) throws IOException {
      IndentingPrintWriter pw = ArtisynthIO.newIndentingPrintWriter (file);
      maspack.util.NumberFormat fmt = new maspack.util.NumberFormat("%g");

      IndentingPrintWriter.printOpening (pw, "[ ");
      IndentingPrintWriter.addIndentation (pw, 2);
      pw.print ("mechmodel=");
      myMech.write (pw, fmt, myMech);
      writeFilePath (pw, "workingFolder=", getWorkingFolder());
      if (myExportFolder != null) {
         writeFilePath (pw, "exportingFolder=", myExportFolder);
      }
      getAllPropertyInfo().writeProps (this, pw, fmt, this);
      IndentingPrintWriter.addIndentation (pw, -2);
      pw.println ("]");

      // write (pw, new maspack.util.NumberFormat("%g"), this);
      pw.close();
   }

   MechModel scanningMech;

   /**
    * Reads a single property or attribute from a case file.
    */
   protected boolean loadItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {
      // if keyword is a property name, try scanning that
      rtok.nextToken();
      if (scanAttributeName (rtok, "mechmodel")) {
         scanningMech = new MechModel();
         scanningMech.scan (rtok, tokens);
         return true;
      }
      else if (scanAttributeName (rtok, "workingFolder")) {
         setWorkingFolder (new File (rtok.scanQuotedString('"')));
         return true;
      }
      else if (scanAttributeName (rtok, "exportFolder")) {
         setExportFolder (new File (rtok.scanQuotedString('"')));
         return true;
      }
      else {
         if (ScanWriteUtils.scanProperty (rtok, this, tokens)) {
            return true;
         }
      }
      rtok.pushBack();
      return false;
   }

   /**
    * Loads a case from a file, by reading its MechModel and root model
    * properties.
    */
   public void loadCase (File file) throws IOException {
      scanningMech = null;
      myTaskFrame.clearPropDialogs();
      DonorType prevDonorType = getDonorType();
      ReaderTokenizer rtok = null;
      try {
         rtok = ArtisynthIO.newReaderTokenizer (file);         
         Deque<ScanToken> tokens = new ArrayDeque<>();
         setScanning (true);
         rtok.scanToken ('[');
         while (rtok.nextToken() != ']') {
            rtok.pushBack();
            if (!loadItem (rtok, tokens)) {
               throw new IOException (
                  "Error scanning " + getClass().getName() +
                  ": unexpected token: " + rtok);
            }
         }

         if (scanningMech != null) {
            scanningMech.postscan (tokens, scanningMech);
            ScanWriteUtils.recursivelyConnectComponents (scanningMech);
            removeModel (myMech);
            addModel (scanningMech);

            // after the new MechModel has been read in, it is necessary to
            // reset all references to its components within the GUI window and
            // elsewhere:

            myMech = scanningMech;
 
            myMeshManager =
               (MeshManager)myMech.get("meshManager");
            myMeshManager.initialize();

            mySegmentGenerator =
               (SegmentGenerator)myMech.get("segmentGenerator");
            mySegmentGenerator.initialize();

            myImplantsManager =
               (ImplantsManager)myMech.get("implantsManager");
            myImplantsManager.initialize();

            myDicomManager =
               (DicomManager)myMech.get("dicomManager");
            myDicomManager.initialize();

            myDonorGuideBuilder =
               (DonorGuideBuilder)myMech.get("donorGuideBuilder");
            myDonorGuideBuilder.initialize();

            myMandibleGuideBuilder =
               (MandibleGuideBuilder)myMech.get("mandibleGuideBuilder");
            myMandibleGuideBuilder.initialize();

            myPlateBuilder = (PlateBuilder)myMech.get("plateBuilder");
            myPlateBuilder.initialize();

            myPlateMarkers =
               (PointList<FrameMarker>)myMech.get("plateMarkers");
            myAngleMarkers =
               (PointList<FrameMarker>)myMech.get("angleMarkers");
            myDonorMarkers =
               (PointList<FrameMarker>)myMech.get("donorMarkers");
            myDeformMarkers =
               (PointList<FrameMarker>)myMech.get("deformMarkers");
            myImplants =
               (RenderableComponentList<Implant>)myMech.get("implants");
            myRDPLineFrames =
               (RenderableComponentList<RigidBody>)myMech.get("RDPLineFrames");
            myRDPLinePoints =
               (PointList<FrameMarker>)myMech.get("RDPLinePoints");
            myRDPLineSegments = 
               (AxialSpringList<AxialSpring>)myMech.get("RDPLineSegments");
            myMandibleHandlePoints = 
               (PointList<Point>)myMech.get("mandibleHandlePoints");
            myDonorSegments =
               (RenderableComponentList<DonorSegmentBody>)
               myMech.get("donorSegments");
            myResectionPlanes =
               (RenderableComponentList<FixedMeshBody>)
               myMech.get("resectionPlanes");
            myTrimPlanes =
               (RenderableComponentList<FixedMeshBody>)
               myMech.get("trimPlanes");
            myScrews =
               (RenderableComponentList<FixedMeshBody>)
               myMech.get("screws");

            myTaskFrame.initReferences();
            if (prevDonorType != getDonorType()) {
               myTaskFrame.createOrUpdateSubPanels();
               myTaskFrame.updateButtons();
               myTaskFrame.pack();
            }
            else {
               myTaskFrame.updatePropPanels();
            }
         }
      }
      catch (Exception e) {
         throw e;
      }
      finally {
         if (rtok != null) {
            rtok.close();
         }
      }
      setScanning (false);
   }

   public void componentChanged (ComponentChangeEvent e) {
      super.componentChanged(e);
      if (e.getCode() == ComponentChangeEvent.Code.GEOMETRY_CHANGED) {
         ModelComponent comp = e.getComponent();
         if (comp instanceof FixedMeshBody) {
            mySegmentGenerator.cutPlaneGeometryChanged ((FixedMeshBody)comp);
            if ("planeL".equals (comp.getName()) ||
                "planeR".equals (comp.getName())) {
               myMeshManager.invalidateResectionMesh();
//                if (myMeshManager.hasClippedMandible()) {
//                   myMeshManager.createClippedMandible();
//                }
//                if (myTaskFrame.hasReconstruction()) {
//                   myTaskFrame.clearReconstruction();
//                }
             }
         }
         if (comp instanceof Point &&
             myMandibleGuideBuilder.getHandlePoints().contains(comp)) {
            myMandibleGuideBuilder.updateHandle();
         }
      }
   }


}
