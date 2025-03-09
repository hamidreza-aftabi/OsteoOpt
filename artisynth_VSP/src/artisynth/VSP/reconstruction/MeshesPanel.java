package artisynth.VSP.reconstruction;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JPanel;

import artisynth.VSP.reconstruction.ReconAppRoot.DonorType;
import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.FrameMarkerAgent;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import maspack.properties.HasProperties;
import maspack.render.RenderProps;
import maspack.widgets.GuiUtils;
import maspack.widgets.PropertyPanel;

/**
 * GUI tab for importing mesh and cut plane info and preparing it for the
 * reconstruction planning.
 */
public class MeshesPanel extends TabPanel {

   // references to the root model and worker components:
   MeshManager myMeshManager;
   PlateBuilder myPlateBuilder;
   
   // Buttons for the button panel. We keep references to these so that we can
   // enable or disable buttons depending on the state of the workflow.
   private JButton myAddPlateMarkersButton;
   private JButton myAddAngleMarkersButton;
   private JButton myCreateResectionPlanesButton;
   private JButton myClipMandibleButton;
   private JButton myClipDonorButton;

   /**
    * Creates the panel containing buttons to perform various tasks for
    * creating and editing reconstruction objects.
    */
   JPanel createTaskPanel () {
      
      JPanel panel = new JPanel();
      panel.setLayout (new BoxLayout (panel, BoxLayout.PAGE_AXIS));
      DonorType donorType = myRoot.getDonorType();      

      myAddPlateMarkersButton = addVerticalButton (
         panel, "Add plate markers",
         "Add markers to the mandible to delimit the plate",
         e->addPlateMarkers());

      myAddAngleMarkersButton = addVerticalButton (
         panel, "Add angle markers",
         "Add angle markers to the mandible",
         e->addAngleMarkers());

      myCreateResectionPlanesButton = addVerticalButton (
         panel, "Create resection planes",
         "Create a new set of resection planes to be positions by the user",
         e->createResectionPlanes());

      myClipMandibleButton = addVerticalButton (
         panel, "Clip mandible",
         "Clip the mandible using the resection planes",
         e->clipMandible());

      if (donorType == DonorType.FIBULA) {
         myClipDonorButton = addVerticalButton (
            panel, "Clip donor",
            "Clip the ends of the fibula",
            e->clipDonor());
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

      host = myPlateBuilder;
      panel.addWidget (host, "markersLeftToRight");

      if (donorType == DonorType.FIBULA) {
         panel.addLabel (" donor preparation:");
         host = myMeshManager;
         panel.addWidget (host, "maxDonorRadius");
         panel.addWidget (host, "proxTrimDistance");
         panel.addWidget (host, "distalTrimDistance");
      }

      addStandardProps (panel);
      return panel;
   }
   
   void initReferences() {
      myMeshManager = myRoot.getMeshManager();
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

      if (donorType == DonorType.FIBULA) {
         updateWidgetsForHost (
            panel, myMeshManager, 
            "maxDonorRadius",
            "proxTrimDistance", 
            "distalTrimDistance");
      }
      updateWidgetsForHost (
         panel, myPlateBuilder,
         "markersLeftToRight");
   }

   /**
    * Creates a new TaskFrame, consisting of a task panel on the left, property
    * panel on the right, and a menu bar.
    */
   public MeshesPanel (TaskFrame taskFrame) {
      super (taskFrame);
   }

   // resection planes

   /**
    * Queries if it is possible to create the resection planes.
    */
   private boolean createResectionPlanesEnabled() {
      return (myTaskFrame.hasMandible());
   }

   /**
    * Create a pair of resection planes in a standard position from which
    * they can be placed the user.
    */
   void createResectionPlanes() {
      myTaskFrame.clearResectionPlanes();
      myMeshManager.createResectionPlanes();
   }

   /**
    * Creates a dialog for adding frame markers to a named rigid body and
    * collecting them within a specified list.
    */
   private void addMarkers (
      PointList<FrameMarker> markerList, String name) {
      Main main = Main.getMain();
      if (main.getEditorManager().acquireEditLock()) {
         FrameMarkerAgent agent =
            new FrameMarkerAgent (
               main, myRoot.getMechModel(),
               markerList,
               myMeshManager.getRigidBody (name));
         java.awt.Rectangle bounds = GuiUtils.getScreenBounds (this);
         agent.show (bounds);
         String title = "Add " + name + " markers";
         agent.getDisplay().setTitle (title);
         myTaskFrame.setEditDialog (title);
      }
      else {
         myTaskFrame.requestEditDialogClose();
      }
   }

   // plate markers

   /**
    * Queries if it is possible to add plate markers.
    */
   private boolean addPlateMarkersEnabled() {
      return myTaskFrame.hasMandible();
   }

   /**
    * Invokes a dialog to add plate markers.
    */
   private void addPlateMarkers() {
      if (!myTaskFrame.checkForMandible()) {
         return;
      }
      addMarkers (myPlateBuilder.getPlateMarkers(), "mandible");
      myTaskFrame.setPlateMarkersVisible (true);
      myTaskFrame.setMandibleVisible (true);
   }

   // angle markers

   /**
    * Queries if it is possible to add angle markers.
    */
   private boolean addAngleMarkersEnabled() {
      return myTaskFrame.hasMandible();
   }

   /**
    * Invokes a dialog to add angle markers.
    */
   private void addAngleMarkers() {
      if (!myTaskFrame.checkForMandible()) {
         return;
      }
      addMarkers (myMeshManager.getAngleMarkers(), "mandible");
      myTaskFrame.setAngleMarkersVisible (true);
      myTaskFrame.setMandibleVisible (true);
      rerender();
   }

   // clipped mandible mesh
   
   /**
    * Queries if it is possible to create the clipped mandible.
    */
   private boolean clipMandibleEnabled() {
      return (myTaskFrame.hasMandible() && myTaskFrame.hasResectionPlanes());
   }

   /**
    * Creates the clipped mandible.
    */
   public void clipMandible() {
      if (!myTaskFrame.checkForMandible() ||
          !myTaskFrame.checkForResectionPlanes()) {
         return;
      }
      try {
         myMeshManager.createClippedMandible();
      }
      catch (Exception e) {
         showError ("Error clipping mandible", e);
         return;
      }
      myTaskFrame.setResectionPlanesVisible (false);
      myTaskFrame.setMandibleVisible (false);
      rerender();
   }

   /**
    * Queries if it is possible to create the clipped donor.
    */
   private boolean clipDonorEnabled() {
      return myTaskFrame.hasDonor();
   }

   /**
    * Creates the clipped donor.
    */
   public void clipDonor () {
      if (!myTaskFrame.checkForDonor()) {
         return;
      }
      RigidBody donor = myMeshManager.getDonor();
      try {
         myMeshManager.createClippedDonor (donor);
      }
      catch (Exception e) {
         showError ("Error clipping donor", e);
         return;
      }
      myTaskFrame.clearDonorMarkers();
      myTaskFrame.clearDonorSegments();
      RenderProps.setVisible (donor, false);
      rerender();
   }

   /**
    * Enables or disables buttons in the task panel depending on the current
    * workflow state.
    */
   public void updateButtons() {
      myAddPlateMarkersButton.setEnabled (addPlateMarkersEnabled());
      myAddAngleMarkersButton.setEnabled (addAngleMarkersEnabled());
      myCreateResectionPlanesButton.setEnabled (createResectionPlanesEnabled());
      myClipMandibleButton.setEnabled (clipMandibleEnabled());
      if (myClipDonorButton != null) {
         myClipDonorButton.setEnabled (clipDonorEnabled());
      }
   }
}
