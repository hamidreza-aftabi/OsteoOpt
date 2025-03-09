package artisynth.VSP.reconstruction;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JPanel;

import artisynth.VSP.reconstruction.ReconAppRoot.DonorType;
import artisynth.core.mechmodels.RigidBody;
import maspack.geometry.PolygonalMesh;
import maspack.properties.HasProperties;
import maspack.widgets.GuiUtils;
import maspack.widgets.PropertyPanel;

/**
 * GUI tab for importing mesh and cut plane info and preparing it for the
 * reconstruction planning.
 */
public class DonorGuidePanel extends TabPanel {

   // references to the root model and worker components:
   MeshManager myMeshManager;
   SegmentGenerator mySegmentGenerator;
   DonorGuideBuilder myDonorGuideBuilder;
   PlateBuilder myPlateBuilder;
   
   // Buttons for the button panel. We keep references to these so that we can
   // enable or disable buttons depending on the state of the workflow.
   private JButton myCreateDonorGuideButton;
   private JButton myAddDonorGuideHolesButton;
   private JButton myClearDonorGuideHolesButton;

   /**
    * Creates the panel containing buttons to perform various tasks for
    * creating and editing reconstruction objects.
    */
   JPanel createTaskPanel () {
      
      JPanel panel = new JPanel();
      panel.setLayout (new BoxLayout (panel, BoxLayout.PAGE_AXIS));
      DonorType donorType = myRoot.getDonorType();      

      myCreateDonorGuideButton = addVerticalButton (
         panel, "Create donor guide",
         "Create donor cutting guide from the segments",
         e->createDonorGuide());

      myAddDonorGuideHolesButton = addVerticalButton (
         panel, "Add donor guide holes",
         "Add screw holes to the donor guide",
         e->addDonorGuideHoles());

      myClearDonorGuideHolesButton = addVerticalButton (
         panel, "Clear donor guide holes",
         "Remove screw holes from the donor guide",
         e->myTaskFrame.clearDonorGuideHoles());

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

      if (donorType == DonorType.FIBULA) {
         host = myDonorGuideBuilder;
         panel.addWidget (host, "setbackFraction");
         panel.addWidget (host, "recessType");
         panel.addWidget (host, "recessClearance");
         panel.addWidget (host, "topFlangeDepth");
         panel.addWidget (host, "minFlangeWidth");
      }

      addStandardProps (panel);
      return panel;
   }

   void initReferences() {
      myMeshManager = myRoot.getMeshManager();
      mySegmentGenerator = myRoot.getSegmentGenerator();
      myPlateBuilder = myRoot.getPlateBuilder();
      myDonorGuideBuilder = myRoot.getDonorGuideBuilder();
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
            panel, myDonorGuideBuilder,
            "setbackFraction",
            "recessType",
            "recessClearance",
            "topFlangeDepth",
            "minFlangeWidth");
      }
   }

   /**
    * Creates a new TaskFrame, consisting of a task panel on the left, property
    * panel on the right, and a menu bar.
    */
   public DonorGuidePanel (TaskFrame taskFrame) {
      
      super (taskFrame);
   }

   // donor guide

   /**
    * Queries if it is possible to create the donor guide.
    */
   boolean createDonorGuideEnabled() {
      return myTaskFrame.hasClippedDonor() && myTaskFrame.hasDonorSegments();
   }

   /**
    * Creates the donor guide.
    */
   void createDonorGuide() {
      if (!myTaskFrame.checkForClippedDonor() ||
          !myTaskFrame.checkForDonorSegments()) {
         return;
      }
      RigidBody clippedDonor = myMeshManager.getRigidBody ("clippedDonor");
      try {
         DonorGuideBuilder builder = myDonorGuideBuilder;
         builder.createFibulaGuide (
            clippedDonor.getSurfaceMesh(),
            myMeshManager.getCurve("donorCurve").getSpline(),
            myMeshManager.getMaxDonorRadius(),
            mySegmentGenerator.getSegments(),
            mySegmentGenerator.getDonorEndPoint());
         //myRoot.setDonorGuide (guide);
         rerender();
      }
      catch (Exception e) {
         showError ("Error computing donor guide", e);
      }
   }

   /**
    * Queries if it is possible to add holes to the donor guide.
    */
   boolean addDonorGuideHolesEnabled() {
      return (myTaskFrame.hasDonorGuide() &&
              myTaskFrame.hasDonorSegments() &&
              (myTaskFrame.hasImplants() || myTaskFrame.hasScrews()));
   }

   /**
    * Queries if it is possible to remove holes from the donor guide.
    */
   boolean clearDonorGuideHolesEnabled() {
      return myTaskFrame.hasDonorGuide() && myTaskFrame.hasDonorGuideHoles();
   }

   /**
    * Add screw holes to the donor guide.
    */
   private void addDonorGuideHoles() {
      if (myTaskFrame.hasScrews()) {
         PolygonalMesh screwMesh =
            myPlateBuilder.getDonorScrewMesh(
               mySegmentGenerator.getSegments(),
               myTaskFrame.hasSelectedScrews());
         if (screwMesh.numVertices() > 0) {
            try {
               DonorGuideBuilder builder = myDonorGuideBuilder;
               builder.addScrewHoles (screwMesh);
               //myRoot.setDonorGuide (mesh);
               rerender();
            }
            catch (Exception e) {
               showError ("Error adding screw holes", e);
            }
         }
         else {
            GuiUtils.showWarning (
               this, "Selected screws do not intersect any donor segments");
         }
      }
      if (myTaskFrame.hasImplants()) {
         try {
            DonorGuideBuilder builder = myDonorGuideBuilder;
            builder.addImplantHoles (
               mySegmentGenerator.getSegments(), myMeshManager.getImplants());
            //myRoot.setDonorGuide (mesh);
            rerender();
         }
         catch (Exception e) {
            showError ("Error adding implant holes", e);
         }       
      }
   }

   /**
    * Enables or disables buttons in the task panel depending on the current
    * workflow state.
    */
   public void updateButtons() {
      myCreateDonorGuideButton.setEnabled (createDonorGuideEnabled());
      myAddDonorGuideHolesButton.setEnabled (addDonorGuideHolesEnabled());
      myClearDonorGuideHolesButton.setEnabled (clearDonorGuideHolesEnabled());
   }

}
