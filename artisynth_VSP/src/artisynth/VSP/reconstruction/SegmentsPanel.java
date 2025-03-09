package artisynth.VSP.reconstruction;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JPanel;

import artisynth.VSP.reconstruction.ReconAppRoot.DonorType;
import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.FrameMarkerAgent;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.RenderableComponentList;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.properties.HasProperties;
import maspack.widgets.GuiUtils;
import maspack.widgets.PropertyPanel;

/**
 * GUI tab for importing mesh and cut plane info and preparing it for the
 * reconstruction planning.
 */
public class SegmentsPanel extends TabPanel {

   // references to the root model and worker components:
   MeshManager myMeshManager;
   SegmentGenerator mySegmentGenerator;
   PlateBuilder myPlateBuilder;
   
   // Buttons for the button panel. We keep references to these so that we can
   // enable or disable buttons depending on the state of the workflow.
   private JButton myPlateButton;
   private JButton myRDPSimpButton;
   private JButton myAddDonorMarkersButton;
   private JButton myFindDonorSegmentsButton;
   private JButton myCreateReconstructionButton;
   private JButton myTestDonorMarchButton;

   /**
    * Creates the panel containing buttons to perform various tasks for
    * creating and editing reconstruction objects.
    */
   JPanel createTaskPanel () {
      
      JPanel panel = new JPanel();
      panel.setLayout (new BoxLayout (panel, BoxLayout.PAGE_AXIS));
      DonorType donorType = myRoot.getDonorType();      

      myPlateButton = addVerticalButton (
         panel, "Create plate curve",
         "Create the plate curve from the plate markers",
         e->createPlateCurve());

      myRDPSimpButton = addVerticalButton (
         panel, "Create RDP line",
         "Simplify the plate using RDP",
         e->createRDPLine());
      
      myAddDonorMarkersButton = addVerticalButton (
         panel, "Add donor marker(s)",
         "Add marker(s) to the donor",
         e->addDonorMarkers());

      myFindDonorSegmentsButton = addVerticalButton (
         panel, "Find donor segments",
         "Find cut planes and corresponding segments on the donor",
         e->findDonorSegments());

      myCreateReconstructionButton = addVerticalButton (
         panel, "Create reconstruction",
         "Create full mesh of the mandible reconstruction",
         e->createReconstruction());

      if (donorType == DonorType.FIBULA) {
         myTestDonorMarchButton = createVerticalButton (
            panel, "Test donor march",
            "Test donor march",
            e->testDonorMarch());
      }

      panel.setBorder (
         BorderFactory.createCompoundBorder (
            BorderFactory.createEtchedBorder (),
            BorderFactory.createEmptyBorder (2, 2, 2, 2)));

      return panel;
   }

   /**
    * Creates a panel that allows editing of various propertues related to
    * reconstruction objects.
    */
   PropertyPanel createPropPanel() {
      PropertyPanel panel = new PropertyPanel();
      DonorType donorType = myRoot.getDonorType();      

      HasProperties host;

      panel.addLabel (" RDP line:");
      host = mySegmentGenerator;
      panel.addWidget (host, "minSegLength");
      panel.addWidget (host, "maxSegments");
      panel.addWidget (host, "numSegments");

      if (donorType == DonorType.FIBULA) {
         panel.addLabel (" segment generation:");
         host = mySegmentGenerator;
         panel.addWidget (host, "segmentSeparation");
         panel.addWidget (host, "createLeftToRight");
      }
      else { // donor == SCAPULA
         panel.addLabel (" segment generation:");
         host = mySegmentGenerator;
         panel.addWidget (host, "segmentSeparation");
         panel.addWidget (host, "createLeftToRight");
         panel.addWidget (host, "scapulaOrientation");
         panel.addWidget (host, "scapulaHeightReduction");
      }

      addStandardProps (panel);
      return panel;
   }

   void initReferences() {
      myMeshManager = myRoot.getMeshManager();
      mySegmentGenerator = myRoot.getSegmentGenerator();
      myPlateBuilder = myRoot.getPlateBuilder();
   }

   /**
    * Called whenever a new case is loaded to connect the widgets in the
    * property panel to their (newly created) host components.
    */
   void updatePropPanel() {
      PropertyPanel panel = myPropPanel;
      HasProperties host;
      DonorType donorType = myRoot.getDonorType();      

      updateWidgetsForHost (
         panel, mySegmentGenerator,
         "minSegLength",
         "maxSegments",
         "numSegments");

      if (donorType == DonorType.FIBULA) {
         updateWidgetsForHost (
            panel, mySegmentGenerator,
            "segmentSeparation",
            "createLeftToRight");
      }
      else { // donor == SCAPULA
         updateWidgetsForHost (
            panel, mySegmentGenerator,
            "segmentSeparation",
            "createLeftToRight",
            "scapulaOrientation",
            "scapulaHeightReduction");
      }
   }

   /**
    * Creates a new TaskFrame, consisting of a task panel on the left, property
    * panel on the right, and a menu bar.
    */
   public SegmentsPanel (TaskFrame taskFrame) {
      
      super (taskFrame);
   }
   // plate curve

   /**
    * Queries if it is possible to create the plate curve.
    */
   private boolean createPlateCurveEnabled() {
      return myTaskFrame.hasPlateMarkers();
   }

   /**
    * Creates the plate curve.
    */
   public void createPlateCurve() {
      if (!myTaskFrame.checkForPlateMarkers()) {
         return;
      }
      myPlateBuilder.createPlateCurve ();
      myTaskFrame.clearRDPLine();
      rerender();
   }

   /**
    * Convenience method used for making videos
    */
   void createPlateCurveAndRDPLine() {
      createPlateCurve();
      createRDPLine();
   }
   
   // RDP line

   /**
    * Queries if it is possible to create the RDP line.
    */
   private boolean createRDPLineEnabled() {
      return (myTaskFrame.hasPlateCurve() && myTaskFrame.hasResectionPlanes());
   }

   /**
    * Creates the RDP line.
    */
   public void createRDPLine() {
      if (!myTaskFrame.checkForPlateCurve() || 
          !myTaskFrame.checkForResectionPlanes() ||
          !myTaskFrame.checkForMandible()) {
         return;
      }
      RigidTransform3d[] TPWs = myMeshManager.getResectionPlanes();
      mySegmentGenerator.createRDPLine (
         (PolylineMesh)myPlateBuilder.getPlateCurve().getMesh(),
         myMeshManager.getMandibleMesh(),
         new Plane(TPWs[0]),
         new Plane(TPWs[1]));
      myTaskFrame.clearDonorSegments();
      rerender();
   }

   // donor markers

   /**
    * Queries if it is possible to add donor markers.
    */
   private boolean addDonorMarkersEnabled() {
      if (myRoot.getDonorType() == DonorType.SCAPULA) {
          return myTaskFrame.hasDonor();
      }
      else {
          return myTaskFrame.hasClippedDonor();
      }
   }

   /**
    * Invokes a dialog to add donor markers.
    */
   private void addDonorMarkers() {
      RigidBody donor = null;
      if (myRoot.getDonorType() == DonorType.SCAPULA) {
         donor = myMeshManager.getDonor();
      }
      else {
         donor = myMeshManager.getRigidBody("clippedDonor");
      }
      if (donor == null) {
         return;
      }
      Main main = Main.getMain();
      if (main.getEditorManager().acquireEditLock()) {
         FrameMarkerAgent agent =
            new FrameMarkerAgent (
               main, myRoot.getMechModel(),
               myMeshManager.getDonorMarkers(), donor);
         java.awt.Rectangle bounds = GuiUtils.getScreenBounds (this);
         agent.show (bounds);
         if (myRoot.getDonorType() == DonorType.SCAPULA) {
            myTaskFrame.setDonorVisible (true);
         }
         else {
            myTaskFrame.setClippedDonorVisible (true);
         }
         myTaskFrame.clearDonorSegments();
         String title = "Add donor markers";
         agent.getDisplay().setTitle (title);
         myTaskFrame.setEditDialog (title);
      }     
      else {
         myTaskFrame.requestEditDialogClose();
      }
   }

   // donor segments

   /**
    * Queries if it is possible to create the donor segments.
    */
   public boolean findDonorSegmentsEnabled() {
      if (myRoot.getDonorType() == DonorType.SCAPULA) {
         return (
            myTaskFrame.hasDonor() &&
            myTaskFrame.hasMandible() &&
            myTaskFrame.hasPlateMarkers() &&
            myTaskFrame.numDonorMarkers() > 1 &&
            myTaskFrame.hasRDPLine() &&
            myTaskFrame.hasResectionPlanes());
      }
      else {
         return (
            myTaskFrame.hasClippedDonor() &&
            myTaskFrame.hasMandible() &&
            myTaskFrame.hasDonorMarkers() &&
            myTaskFrame.hasRDPLine() &&
            myTaskFrame.hasResectionPlanes());         
      }
   }

   /**
    * Creates the donor segments.
    */
   void findDonorSegments() {
      if (myRoot.getDonorType() == DonorType.SCAPULA) {
         if (!myTaskFrame.checkForDonor() ||
             !myTaskFrame.checkForPlateMarkers() ||
             !myTaskFrame.checkForMandible() ||
             !myTaskFrame.checkForRDPLine() ||
             !myTaskFrame.checkForResectionPlanes() ||
             !myTaskFrame.checkForDonorMarkers(2)) {
            return;
         }
         RigidBody donor = myMeshManager.getDonor();
         PointList<FrameMarker> donorMarkers = myMeshManager.getDonorMarkers();
         PointList<FrameMarker> plateMarkers = myPlateBuilder.getPlateMarkers();
         SegmentGenerator seggen = mySegmentGenerator;
         RenderableComponentList<FixedMeshBody> pbodies =
            myMeshManager.getResectionPlaneBodies();         
         try {
            seggen.createScapulaSegments (
               donorMarkers, plateMarkers, donor,
               myMeshManager.getMandible(),
               pbodies.get(0),  // planeL
               pbodies.get(1)); // planeR
            myTaskFrame.clearDonorGuide();
            myTaskFrame.clearReconstruction();
            //setDonorSegmentsVisible (true);
            rerender();
         }
         catch (Exception e) {
            showError ("Error computing cut planes", e);
         }
      }
      else {
         if (!myTaskFrame.checkForClippedDonor() ||
             !myTaskFrame.checkForMandible() ||
             !myTaskFrame.checkForRDPLine() ||
             !myTaskFrame.checkForResectionPlanes() ||
             !myTaskFrame.checkForDonorMarkers(1)) {
            return;
         }
         RigidBody clippedDonor = myMeshManager.getRigidBody ("clippedDonor");
         PointList<FrameMarker> donorMarkers = myMeshManager.getDonorMarkers();

         SegmentGenerator seggen = mySegmentGenerator;
         RenderableComponentList<FixedMeshBody> pbodies =
            myMeshManager.getResectionPlaneBodies();         
         try {
            seggen.createFibulaSegments (
               donorMarkers.get(0).getPosition(), clippedDonor,
               myMeshManager.getCurve("donorCurve").getSpline(),
               myMeshManager.getMandibleMesh(),
               pbodies.get(0),  // "planeL"
               pbodies.get(1)); // "planeR"
            seggen.alignSegments (seggen.getCreateLeftToRight());
            myTaskFrame.clearDonorGuide();
            myTaskFrame.clearReconstruction();
            myTaskFrame.setDonorSegmentsVisible (true);
            rerender();
         }
         catch (Exception e) {
            showError ("Error computing cut planes", e);
         }
      }
   }

   /**
    * Convenience method used for making videos.
    */
   void findDonorSegmentsAndReconstruction() {
      findDonorSegments();
      createReconstruction();
   }

   // reconstructed mandible mesh

   /**
    * Queries if it is possible to create the reconstructed mandible.
    */
   private boolean createReconstructionEnabled() {
      return myTaskFrame.hasClippedMandible() && myTaskFrame.hasDonorSegments();
   }

   /**
    * Creates the reconstructed mandible.
    */
  public  void createReconstruction() {
      if (!myTaskFrame.checkForClippedMandible() ||
          !myTaskFrame.checkForDonorSegments()) {
         return;
      }
      RigidBody mandibleL = myMeshManager.getRigidBody ("mandibleL");
      RigidBody mandibleR = myMeshManager.getRigidBody ("mandibleR");
      try {
         PolygonalMesh mesh = myMeshManager.computeReconstruction (
            mandibleL.getSurfaceMesh(),
            mandibleR.getSurfaceMesh(),
            mySegmentGenerator.getSegments());
         myMeshManager.setRigidBody ("reconstruction", mesh);
         myTaskFrame.setReconstructionVisible (false);
         myTaskFrame.clearPlateFem();
      }
      catch (Exception e) {
         showError ("Error computing reconstruction", e);
      }
   }

   // donor march command

   /**
    * Queries if it is possible to test the donor march.
    */
   boolean testDonorMarchEnabled() {
      return (myTaskFrame.hasClippedDonor() && myTaskFrame.hasDonorMarkers());
   }
   
   /**
    * The the donor march by advancing the most recently added donor marker
    * along the length of the donor by the distance specified by the
    * current segment separation.
    */
   private void testDonorMarch() {
      if (!myTaskFrame.checkForClippedDonor() ||
          !myTaskFrame.checkForDonorMarkers(1)) {
         return;
      }
      RigidBody clippedDonor = myMeshManager.getRigidBody ("clippedDonor");
      PointList<FrameMarker> donorMarkers = myMeshManager.getDonorMarkers();
      Spline3dBody spline = myMeshManager.getCurve("donorCurve");
      SegmentGenerator seggen = mySegmentGenerator;
      try {
         Point3d pnt = seggen.findNextDonorPoint (
            donorMarkers.get(donorMarkers.size()-1).getPosition(),
            seggen.getSegmentSeparation(),
            clippedDonor.getSurfaceMesh(), spline.getSpline());
         myMeshManager.addDonorMarker (pnt);      
      }
      catch (Exception e) {
         showError ("Can't compute next point", e);
         return;
      }
   }

   /**
    * Enables or disables buttons in the task panel depending on the current
    * workflow state.
    */
   public void updateButtons() {
      myPlateButton.setEnabled (createPlateCurveEnabled());
      myRDPSimpButton.setEnabled (createRDPLineEnabled());
      myAddDonorMarkersButton.setEnabled (addDonorMarkersEnabled());
      myFindDonorSegmentsButton.setEnabled (findDonorSegmentsEnabled());
      myCreateReconstructionButton.setEnabled (createReconstructionEnabled());
      if (myRoot.getDonorType() == DonorType.FIBULA) {
         myTestDonorMarchButton.setEnabled (testDonorMarchEnabled());
      }
   }

}
