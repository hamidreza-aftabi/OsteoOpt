package artisynth.VSP.reconstruction;

import java.awt.Component;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JPanel;

import artisynth.VSP.reconstruction.ReconAppRoot.DonorType;
import artisynth.core.driver.Main;
import artisynth.core.renderables.DicomPlaneViewer;
import artisynth.core.renderables.DicomViewer;
import maspack.properties.HasProperties;
import maspack.widgets.GuiUtils;
import maspack.widgets.*;
import maspack.matrix.*;

/**
 * GUI tab for importing mesh and cut plane info and preparing it for the
 * reconstruction planning.
 */
public class ImplantsPanel extends TabPanel {

   // references to the root model and worker components:
   MeshManager myMeshManager;
   ImplantsManager myImplantsManager;
   PlateBuilder myPlateBuilder;
   
   // Buttons for the button panel. We keep references to these so that we can
   // enable or disable buttons depending on the state of the workflow.
   private JButton myOcclusalPlaneButton;
   private JButton myOcclusalSliceButton;
   private JButton mySagittalSliceButton;
   private JButton myAddImplantsButton;
   private JButton myClearImplantsButton;

   DicomViewer myDicomViewer;
   DicomPlaneViewer mySlicePlane;

   /**
    * Creates the panel containing buttons to perform various tasks for
    * creating and editing reconstruction objects.
    */
   JPanel createTaskPanel () {
      
      JPanel panel = new JPanel();
      panel.setLayout (new BoxLayout (panel, BoxLayout.PAGE_AXIS));
      DonorType donorType = myRoot.getDonorType();      

      myOcclusalPlaneButton = addVerticalButton (
         panel, "StubText", "StubText", e->occlusalPlaneAction());
      updateOcclusalPlaneButton();

      myOcclusalSliceButton = addVerticalButton (
         panel, "StubText", "StubText", e->occlusalSliceAction());
      updateOcclusalSliceButton();

      mySagittalSliceButton = addVerticalButton (
         panel, "StubText", "StubText", e->sagittalSliceAction());
      updateSagittalSliceButton();

      myAddImplantsButton = addVerticalButton (
         panel, "Add implants",
         "Add implant locations to the mandible",
         e->addImplants());

      myClearImplantsButton = addVerticalButton (
         panel, "Clear implants",
         "Clear all exisiting implant locations",
         e->clearImplants());

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

      host = myImplantsManager;
      panel.addWidget (host, "upperImplantRadius");
      panel.addWidget (host, "lowerImplantRadius");
      panel.addWidget (host, "implantLength");
      panel.addWidget (host, "implantTopExtension");
      panel.addWidget (host, "occlusalOffset");
      panel.addWidget (host, "occlusalSliceContours");
      panel.addWidget (host, "planeSliceScale");

      host = myTaskFrame;
      panel.addWidget (host, "occlusalPlaneVisible");
      panel.addWidget (host, "occlusalSliceVisible");
      panel.addWidget (host, "sagittalSliceVisible");
      panel.addWidget (host, "mandibleVisible");
      panel.addWidget (host, "maxillaVisible");

      addStandardProps (panel);
      return panel;
   }

   void initReferences() {
      myMeshManager = myRoot.getMeshManager();
      myPlateBuilder = myRoot.getPlateBuilder();
      myImplantsManager = myRoot.getImplantsManager();
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
         panel, myImplantsManager,
         "upperImplantRadius",
         "lowerImplantRadius",
         "implantLength",
         "implantTopExtension",
         "occlusalOffset",
         "occlusalSliceContours",
         "planeSliceScale");

      updateWidgetsForHost (
         panel, myTaskFrame,
         "occlusalPlaneVisible",
         "occlusalSliceVisible",
         "sagittalSliceVisible",
         "mandibleVisible",
         "maxillaVisible");
   }

   /**
    * Creates a new TaskFrame, consisting of a task panel on the left, property
    * panel on the right, and a menu bar.
    */
   public ImplantsPanel (TaskFrame taskFrame) {
      
      super (taskFrame);
   }

   // implants

   /**
    * Queries if it is possible to add implants.
    */
   boolean addImplantsEnabled() {
      return myTaskFrame.hasMandible() && myTaskFrame.hasOcclusalPlane();
   }

   /**
    * Invokes a dialog to add angle markers.
    */
   private void addImplants() {
      if (!myTaskFrame.checkForMandible() ||
          !myTaskFrame.hasOcclusalPlane()) {
         return;
      }
      //myTaskFrame.setMandibleVisible (true);
      Main main = Main.getMain();
      if (main.getEditorManager().acquireEditLock()) {
         AddImplantAgent agent =
            new AddImplantAgent (
               main, myRoot.getMechModel(),
               myMeshManager,
               myImplantsManager,
               myPlateBuilder.getPlateMarkers(),
               myTaskFrame.getOcclusalSlice());
         java.awt.Rectangle bounds = GuiUtils.getScreenBounds (this);
         agent.show (bounds);
         String title = "Add implants";
         agent.getDisplay().setTitle (title);
         myTaskFrame.setEditDialog (title);
      }
      else {
         myTaskFrame.requestEditDialogClose();
      }


      //addMarkers (myMeshManager.getImplants(), "mandible");
      myTaskFrame.setImplantsVisible (true);
      rerender();
   }

   /**
    * Queries if it is possible to clear implants.
    */
   boolean clearImplantsEnabled() {
      return myTaskFrame.hasImplants();
   }

   /**
    * Clears all implants
    */
   private void clearImplants() {
      myTaskFrame.clearImplants();
   }

   /**
    * Queries if it is possible to add the occlusal plane.
    */
   boolean addOcclusalPlaneEnabled() {
      return myTaskFrame.hasMandible();
   }

   /**
    * Invokes a dialog to add the occlusal plane.
    */
   private void occlusalPlaneAction() {
      if (myTaskFrame.hasOcclusalPlane()) {
         myTaskFrame.clearOcclusalPlane();         
      }
      else {
         if (!myTaskFrame.checkForMandible()) {
            return;
         }
         myTaskFrame.setMandibleVisible (true);
         Main main = Main.getMain();
         if (main.getEditorManager().acquireEditLock()) {
            AddOcclusalPlaneAgent agent =
               new AddOcclusalPlaneAgent (
                  main, myRoot.getMechModel(), myMeshManager, myImplantsManager);
            java.awt.Rectangle bounds = GuiUtils.getScreenBounds (this);
            agent.show (bounds);
            myTaskFrame.setEditDialog ("Add occlusal plane");
         }
         else {
            myTaskFrame.requestEditDialogClose();
         }
         // //addMarkers (myMeshManager.getImplants(), "mandible");
         myTaskFrame.setOcclusalPlaneVisible (true);
      }
      //updateOcclusalPlaneButton();
      rerender();
   }

   private void updateOcclusalPlaneButton() {
      if (!myTaskFrame.hasOcclusalPlane()) {
         myOcclusalPlaneButton.setText ("Add occlusal plane");
         myOcclusalPlaneButton.setToolTipText (
            "Add occulsal plane to assist with implant placement");
      }
      else {
         myOcclusalPlaneButton.setText ("Clear occlusal plane");
         myOcclusalPlaneButton.setToolTipText (
            "Remove occlusal plane");
      }
   }

   // void createOcclusalSlice (RigidTransform3d TSW) {
   //    if (myDicomViewer == null) {
   //       return;
   //    }
   //    clearOcclusalSlice();

   //    DicomImage image = myDicomViewer.getImage();

   //    Vector2d size = new Vector2d(100, 100);
   //    Point3d cent = myMeshManager.getMandibleCentroid();
   //    RigidTransform3d TVI = null;
   //    if (cent != null) {
   //       TVI = new RigidTransform3d();
   //       TVI.p.set (cent);
   //       TVI.invert();
   //    }
   //    //size.scale (0.5);
   //    DicomPlaneViewer dpv =
   //       new DicomPlaneViewer("occlusalSlice", image, size, TSW, TVI);

   //    RenderProps.setFaceStyle (dpv, FaceStyle.FRONT_AND_BACK);
   //    myRoot.getMechModel().addRenderable(dpv);
   //    myOcclusalSlice = dpv;
   // }

   /**
    * Queries if it is possible to add slices slice.
    */
   boolean addSliceEnabled() {
      return myTaskFrame.hasOcclusalPlane();
   }


   /**
    * Queries if the occlusal slice button should be enabled.
    */
   boolean occlusalSliceButtonEnabled() {
      return myTaskFrame.hasOcclusalPlane();
   }


   /**
    * Adds a dicom slice the occlusal plane.
    */
   private void occlusalSliceAction() {
      if (myTaskFrame.hasOcclusalSlice()) {
         myTaskFrame.clearOcclusalSlice();
      }
      else {
         if (!myTaskFrame.checkForMandible() ||
             !myTaskFrame.hasOcclusalPlane()) {
            return;
         }
         myTaskFrame.createOcclusalSlice (
            myMeshManager.getOcclusalPlane().getPose());
      }
      //updateOcclusalSliceButton();
      rerender();
   }

   private void updateOcclusalSliceButton() {
      if (!myTaskFrame.hasOcclusalSlice()) {
         myOcclusalSliceButton.setText ("Add occlusal slice");
         myOcclusalSliceButton.setToolTipText (
            "Add DICOM view in the occulsal plane to assist with implant placement");
      }
      else {
         myOcclusalSliceButton.setText ("Clear occlusal slice");
         myOcclusalSliceButton.setToolTipText (
            "Remove DICOM view in the occlusal plane");
      }
   }

   /**
    * Adds a dicom slice the sagittal plane.
    */
   private void sagittalSliceAction() {
      if (myTaskFrame.hasSagittalSlice()) {
         myTaskFrame.clearSagittalSlice();
      }
      else {
         if (!myTaskFrame.checkForMandible() ||
             !myTaskFrame.hasOcclusalPlane()) {
            return;
         }
         myTaskFrame.createSagittalSlice (
            myMeshManager.getOcclusalPlane().getPose());
      }
      //updateSagittalSliceButton();
      rerender();
   }

   private void updateSagittalSliceButton() {
      if (!myTaskFrame.hasSagittalSlice()) {
         mySagittalSliceButton.setText ("Add sagittal slice");
         mySagittalSliceButton.setToolTipText (
            "Add DICOM view in the occulsal plane to assist with implant placement");
      }
      else {
         mySagittalSliceButton.setText ("Clear sagittal slice");
         mySagittalSliceButton.setToolTipText (
            "Remove DICOM view in the sagittal plane");
      }
   }

   /**
    * Enables or disables buttons in the task panel depending on the current
    * workflow state.
    */
   public void updateButtons() {
      updateOcclusalPlaneButton();
      updateOcclusalSliceButton();
      updateSagittalSliceButton();
      myOcclusalPlaneButton.setEnabled (addOcclusalPlaneEnabled());
      myOcclusalSliceButton.setEnabled (occlusalSliceButtonEnabled());
      mySagittalSliceButton.setEnabled (addSliceEnabled());
      myAddImplantsButton.setEnabled (addImplantsEnabled());
      myClearImplantsButton.setEnabled (clearImplantsEnabled());
   }

   public void updateWidgetValues () {
      super.updateWidgetValues(); 
      PropertyPanel panel = myPropPanel;
      enableWidget (
         panel, "occlusalPlaneVisible", myTaskFrame.hasOcclusalPlane());
      enableWidget (
         panel, "occlusalSliceVisible", myTaskFrame.hasOcclusalSlice());
      enableWidget (
         panel, "sagittalSliceVisible", myTaskFrame.hasSagittalSlice());
      enableWidget (
         panel, "mandibleVisible", myTaskFrame.hasMandible());
      enableWidget (
         panel, "maxillaVisible", myTaskFrame.hasMaxilla());
   }



}
