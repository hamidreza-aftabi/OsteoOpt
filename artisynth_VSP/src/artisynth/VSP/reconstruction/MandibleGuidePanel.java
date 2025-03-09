package artisynth.VSP.reconstruction;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JButton;

import artisynth.VSP.reconstruction.ReconAppRoot.DonorType;

import javax.swing.*;

import maspack.geometry.PolylineMesh;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.properties.HasProperties;
import maspack.widgets.PropertyPanel;

/**
 * GUI tab for importing mesh and cut plane info and preparing it for the
 * reconstruction planning.
 */
public class MandibleGuidePanel extends TabPanel {

   // references to the root model and worker components:
   MeshManager myMeshManager;
   MandibleGuideBuilder myMandibleGuideBuilder;
   PlateBuilder myPlateBuilder;
   
   // Buttons for the button panel. We keep references to these so that we can
   // enable or disable buttons depending on the state of the workflow.
   private JButton myCreateMandibleGuideButton;
   private JButton myCreateCutBoxesButton;
   private JButton myCreateExpandedMandibleButton;
   private JButton myClearExpandedMandibleButton;
   private JButton myAlignToMandibleButton;
   private JButton myAddFlangesButton;
 
   /**
    * Creates the panel containing buttons to perform various tasks for
    * creating and editing reconstruction objects.
    */
   JPanel createTaskPanel () {
      
      JPanel panel = new JPanel();
      panel.setLayout (new BoxLayout (panel, BoxLayout.PAGE_AXIS));
      DonorType donorType = myRoot.getDonorType();      

      myCreateCutBoxesButton = addVerticalButton (
         panel, "Create cut boxes",
         "Create the cut boxes for the cutting guide",
         e->createCutBoxes());

      myAlignToMandibleButton = addVerticalButton (
         panel, "Align to mandible",
         "Align the cut boxes to properly fit the mandible",
         e->alignToMandible());

      myAddFlangesButton = addVerticalButton (
         panel, "Add flanges",
         "Add flange boxes to each guide section",
         e->addFlanges());
      
      myCreateMandibleGuideButton = addVerticalButton (
         panel, "Create mandible guide",
         "Create mandible cutting guide",
         e->createMandibleGuide());

      panel.add (new JSeparator());

      myCreateExpandedMandibleButton = addVerticalButton (
         panel, "Create expanded mandible",
         "Create expanded mandible to be used for cutting guide",
         e->createExpandedMandible());

      myClearExpandedMandibleButton = addVerticalButton (
         panel, "Clear expanded mandible",
         "Clear expanded mandible to be used for cutting guide",
         e->clearExpandedMandible());

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

      HasProperties host = myMandibleGuideBuilder;
      panel.addWidget (host, "mandibleClearance");
      panel.addWidget (host, "bottomMargin");
      // panel.addWidget (host, "screwSeparation");
      // panel.addWidget (host, "screwRadius");
      panel.addWidget (host, "handleRadius");
      panel.addWidget (host, "handleExtension");
      panel.addWidget (new JSeparator());
      panel.addWidget (host, "allPartsVisible");
      panel.addWidget (host, "guideVisible");
      panel.addWidget (host, "handlePointsVisible");

      host = myPlateBuilder;
      panel.addWidget (host, "screwsVisible");

      host = myMeshManager;
      panel.addWidget (host, "expandedMandibleVisible");

      addStandardProps (panel);
      return panel;
   }

   void initReferences() {
      myMeshManager = myRoot.getMeshManager();
      myPlateBuilder = myRoot.getPlateBuilder();
      myMandibleGuideBuilder = myRoot.getMandibleGuideBuilder();
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
         panel, myMandibleGuideBuilder,
         "mandibleClearance",
         "bottomMargin",
         // "screwSeparation",
         // "screwRadius",
         "handleRadius",
         "handleExtension",
         "allPartsVisible",
         "guideVisible",
         "handlePointsVisible",
         "screwsVisible",
         "expandedMandibleVisible");
   }

   public void updateWidgetValues () {
      super.updateWidgetValues(); 
      PropertyPanel panel = myPropPanel;
      enableWidget (
         panel, "partsVisible", myMandibleGuideBuilder.hasSomeParts());
      enableWidget (
         panel, "guideVisible", myMandibleGuideBuilder.hasGuide());
      enableWidget (
         panel, "handlePointsVisible", myMandibleGuideBuilder.hasHandlePoints());
      enableWidget (
         panel, "screwsVisible", myPlateBuilder.hasScrews());
      enableWidget (
         panel, "expandedMandibleVisible", myMeshManager.hasExpandedMandible());
   }

   /**
    * Creates a new TaskFrame, consisting of a task panel on the left, property
    * panel on the right, and a menu bar.
    */
   public MandibleGuidePanel (TaskFrame taskFrame) {
      
      super (taskFrame);
   }

   // mandible cut boxes

   /**
    * Queries if it is possible to create the cut boxes.
    */
   boolean createCutBoxesEnabled() {
      return (myTaskFrame.hasPlateCurve() &&
              myTaskFrame.hasResectionPlanes() &&
              myTaskFrame.hasMandible());
   }

   /**
    * Creates the cut boxes.
    */
   void createCutBoxes() {
      if (!myTaskFrame.checkForPlateCurve() ||
          !myTaskFrame.checkForResectionPlanes() ||
          !myTaskFrame.checkForMandible()) {
         return;
      }
      try {
         MandibleGuideBuilder builder = myMandibleGuideBuilder;
         builder.createCutBoxes (
            myMeshManager.getMandibleMesh(),
            myMeshManager.getResectionPlanes(),
            (PolylineMesh)myPlateBuilder.getPlateCurve().getMesh());
         rerender();
      }
      catch (Exception e) {
         showError ("Error computing mandible guide", e);
      }
   }

   // mandible guide

   /**
    * Queries if it is possible to create the mandible guide.
    */
   boolean createMandibleGuideEnabled() {
      return (myMandibleGuideBuilder.hasCutBoxes() &&
              myMandibleGuideBuilder.hasFlanges());
   }

   /**
    * Creates the mandible guide.
    */
   void createMandibleGuide() {
      if (!createMandibleGuideEnabled()) {
         return;
      }
      try {
         myMandibleGuideBuilder.createGuide();
         rerender();
      }
      catch (Exception e) {
         showError ("Error computing mandible guide", e);
      }
   }

   // expanded mandible

   boolean createExpandedMandibleEnabled() {
      return (myTaskFrame.hasMandible());
   }

   boolean clearExpandedMandibleEnabled() {
      return (myMeshManager.getRigidBody ("expandedMandible") != null);
   }

   boolean alignToMandibleEnabled() {
      return (clearExpandedMandibleEnabled() &&
              myMandibleGuideBuilder.hasCutBoxes());
   }

   /**
    * Creates the mandible guide.
    */
   void createExpandedMandible() {
      if (!myTaskFrame.checkForMandible()) {
         return;
      }
      try {
         PolygonalMesh expandedMesh = 
            MeshFactory.extrudeAlongVertexNormals (
               myMeshManager.getMandibleMesh(),
               myMandibleGuideBuilder.getMandibleClearance());
         myMeshManager.setRigidBody ("expandedMandible", expandedMesh);
         rerender();
      }
      catch (Exception e) {
         showError ("Error computing mandible guide", e);
      }
   }

   /**
    * Clears the expanded mandible.
    */
   void clearExpandedMandible() {
      myMeshManager.removeRigidBody ("expandedMandible");
   }

   void alignToMandible() {
      myMandibleGuideBuilder.alignToMandible();
      rerender();
   }


   // add flange boxes

   boolean addFlangesEnabled() {
      return myMandibleGuideBuilder.hasCutBoxes();
   }

   void addFlanges() {
      myMandibleGuideBuilder.addFlanges(
         (PolylineMesh)myPlateBuilder.getPlateCurve().getMesh());
      rerender();
   }


   /**
    * Enables or disables buttons in the task panel depending on the current
    * workflow state.
    */
   public void updateButtons() {
      myCreateCutBoxesButton.setEnabled (createCutBoxesEnabled());
      myCreateMandibleGuideButton.setEnabled (createMandibleGuideEnabled());
      myCreateExpandedMandibleButton.setEnabled (createExpandedMandibleEnabled());
      myClearExpandedMandibleButton.setEnabled (clearExpandedMandibleEnabled());
      myAlignToMandibleButton.setEnabled (alignToMandibleEnabled());
      myAddFlangesButton.setEnabled (addFlangesEnabled());
   }

}
