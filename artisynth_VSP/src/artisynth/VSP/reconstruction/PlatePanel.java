package artisynth.VSP.reconstruction;

import java.util.Collection;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

import artisynth.VSP.reconstruction.ReconAppRoot.DonorType;
import artisynth.core.driver.Main;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.RigidBody;
import maspack.geometry.PolylineMesh;
import maspack.properties.HasProperties;
import maspack.widgets.GuiUtils;
import maspack.widgets.PropertyPanel;

/**
 * GUI tab for importing mesh and cut plane info and preparing it for the
 * reconstruction planning.
 */
public class PlatePanel extends TabPanel {

   // references to the root model and worker components:
   MeshManager myMeshManager;
   PlateBuilder myPlateBuilder;
   
   // Buttons for the button panel. We keep references to these so that we can
   // enable or disable buttons depending on the state of the workflow.
   private JButton myCreatePlateButton;
   private JButton myCreateScrewsButton;

   /**
    * Creates the panel containing buttons to perform various tasks for
    * creating and editing reconstruction objects.
    */
   JPanel createTaskPanel () {
      
      JPanel panel = new JPanel();
      panel.setLayout (new BoxLayout (panel, BoxLayout.PAGE_AXIS));
      DonorType donorType = myRoot.getDonorType();      

      myCreatePlateButton = addVerticalButton (
         panel, "Create plate FEM",
         "Create the full plate from the markers",
         e->createPlateFem());

      myCreateScrewsButton = addVerticalButton (
         panel, "Create screws",
         "Create the screws from the deformed plate",
         e->createScrews());

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
      panel.addWidget (host, "springStiffness");
      panel.addWidget (host, "numScrews");
      panel.addWidget (host, "screwOffset");

      host = myRoot.getMechModel();
      panel.addWidget (host, "integrator");

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

      updateWidgetsForHost (
         panel, myPlateBuilder,
         "markersLeftToRight",
         "springStiffness",
         "numScrews",
         "screwOffset");

      updateWidgetsForHost (
         panel, myRoot.getMechModel(),
         "integrator");
   }

   /**
    * Creates a new TaskFrame, consisting of a task panel on the left, property
    * panel on the right, and a menu bar.
    */
   public PlatePanel (TaskFrame taskFrame) {
      
      super (taskFrame);
   }

   // plate FEM

   /**
    * Queries if it is possible to create the plate FEM.
    */
   boolean createPlateFemEnabled() {
      return
         myTaskFrame.hasPlateMarkers() &&
         myTaskFrame.hasPlateCurve() &&
         myTaskFrame.hasMandible() &&
         myTaskFrame.hasReconstruction();
   }

   /**
    * Creates the plate FEM.
    */
   public void createPlateFem() {
      if (!myTaskFrame.checkForPlateMarkers() ||
          !myTaskFrame.checkForMandible() ||
          !myTaskFrame.checkForPlateCurve() ||
          !myTaskFrame.checkForReconstruction()) {
         return;
      }
      Collection<FrameMarker> markers = myPlateBuilder.getPlateMarkers();
      if (markers.size() < 6) {
         GuiUtils.showError (
            this, "At least 6 plate markers required to define the plate");
         return;
      }
      RigidBody mandible = myMeshManager.getMandible();
      PlateBuilder builder = myPlateBuilder;
      builder.buildPlateFem (
         markers, 
         (PolylineMesh)myPlateBuilder.getPlateCurve().getMesh(),
         mandible.getSurfaceMesh(),
         myMeshManager.getRigidBody ("reconstruction"),
         /*standoff=*/10);
      myTaskFrame.setPlateFemVisible (true);
      myTaskFrame.clearScrews();
      rerender();
      MechModel mech = myRoot.getMechModel();
      if ((mech.getIntegrator() == Integrator.StaticIncrementalStep) ||
          (mech.getIntegrator() == Integrator.StaticLineSearch)) {
         // take a single time
         SwingUtilities.invokeLater (
            () -> Main.getMain().step());
      }
   }

   /**
    * Queries if it is possible to create the plate screws.
    */
   boolean createScrewsEnabled() {
      return myTaskFrame.hasPlateFem();
   }

   /**
    * Creates the plate screws.
    */      
   public void createScrews() {
      myPlateBuilder.buildScrews();
      myTaskFrame.setScrewsVisible (true);
   }

   /**
    * Enables or disables buttons in the task panel depending on the current
    * workflow state.
    */
   public void updateButtons() {
      myCreatePlateButton.setEnabled (createPlateFemEnabled());
      myCreateScrewsButton.setEnabled (createScrewsEnabled());
   }

}
