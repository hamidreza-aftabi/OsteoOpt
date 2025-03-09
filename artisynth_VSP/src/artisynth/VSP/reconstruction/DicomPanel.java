package artisynth.VSP.reconstruction;

import java.awt.Component;
import java.io.*;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.*;

import artisynth.VSP.reconstruction.ReconAppRoot.DonorType;
import artisynth.core.driver.Main;
import artisynth.core.renderables.DicomPlaneViewer;
import artisynth.core.renderables.DicomViewer;
import maspack.properties.HasProperties;
import maspack.widgets.GuiUtils;
import maspack.widgets.*;
import maspack.matrix.*;
import maspack.render.*;

/**
 * GUI tab for viewing an imported dicom image.
 */
public class DicomPanel extends TabPanel {

   // references to the root model and worker components:
   DicomManager myDicomManager;

   private JFileChooser myDicomFileChooser;
   
   // Buttons for the button panel. We keep references to these so that we can
   // enable or disable buttons depending on the state of the workflow.
   private JButton myImportDicomButton;
   private JButton myCoronalViewButton;
   private JButton mySagittalViewButton;
   private JButton myAxialViewButton;
   private JButton my3DViewButton;

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

      myImportDicomButton = addVerticalButton (
         panel, "Import dicom", "Import dicom", e->importDicomImage());

      myCoronalViewButton = addVerticalButton (
         panel, "Coronal view", "Set coronal view", e->setCoronalView());

      mySagittalViewButton = addVerticalButton (
         panel, "Sagittal view", "Set sagittal view", e->setSagittalView());

      myAxialViewButton = addVerticalButton (
         panel, "Axial view", "Set axial view", e->setAxialView());

      my3DViewButton = addVerticalButton (
         panel, "3D view", "Set 3D view", e->set3DView());

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

      host = myDicomManager;
      panel.addWidget (host, "viewerVisible");
      panel.addWidget (host, "xPosition", 0, 1.0);
      panel.addWidget (host, "yPosition", 0, 1.0);
      panel.addWidget (host, "zPosition", 0, 1.0);
      panel.addWidget (host, "YZVisible");
      panel.addWidget (host, "XZVisible");
      panel.addWidget (host, "XYVisible");

      addStandardProps (panel);
      return panel;
   }

   void initReferences() {
      myDicomManager = myRoot.getDicomManager();
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
         panel, myDicomManager,
         "viewerVisible",
         "xPosition",
         "yPosition",
         "zPosition",
         "YZVisible",
         "XZVisible",
         "XYVisible");
   }

   /**
    * Creates a new TaskFrame, consisting of a task panel on the left, property
    * panel on the right, and a menu bar.
    */
   public DicomPanel (TaskFrame taskFrame) {
      
      super (taskFrame);
   }

   /**
    * Enables or disables buttons in the task panel depending on the current
    * workflow state.
    */
   public void updateButtons() {
      boolean hasDicom = myDicomManager.hasViewer();      
      myImportDicomButton.setEnabled (hasDicom);
      // myCoronalViewButton.setEnabled (hasDicom);
      // mySagittalViewButton.setEnabled (hasDicom);
      // myAxialViewButton.setEnabled (hasDicom);
      // my3DViewButton.setEnabled (hasDicom);
   }

   public void updateWidgetValues () {
      super.updateWidgetValues(); 
      PropertyPanel panel = myPropPanel;
      boolean hasDicom = myDicomManager.hasViewer();
      enableWidget (
         panel, "viewerVisible", hasDicom);
      enableWidget (
         panel, "xPosition", hasDicom);
      enableWidget (
         panel, "yPosition", hasDicom);
      enableWidget (
         panel, "zPosition", hasDicom);

      enableWidget (
         panel, "YZVisible", hasDicom);
      enableWidget (
         panel, "XZVisible", hasDicom);
      enableWidget (
         panel, "XYVisible", hasDicom);
   }

   /**
    * Selects a file for importing the dicom image.
    */
   File selectDicomFile (String approveMsg) {
      JFileChooser chooser = myDicomFileChooser;
      if (chooser == null) {
         chooser = new JFileChooser();
         chooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
         chooser.setCurrentDirectory (myRoot.getWorkingFolder());
         myDicomFileChooser = chooser;
      }
      int retval = myDicomFileChooser.showDialog (this, approveMsg);
      if (retval == JFileChooser.APPROVE_OPTION) {
         return TaskFrame.cleanPath (myDicomFileChooser.getSelectedFile());
      }
      else {
         return null;
      }
   }      

   /**
    * Imports the dicom image from a user-selected file.
    */
   void importDicomImage () {
      File file = selectDicomFile ("Import");
      if (file != null) {
         try {
            myDicomManager.createViewer (file);
         }
         catch (Exception e) {
            showError ("Error reading file "+file, e);
         }
      }
   }

   void setSagittalView() {
      Viewer viewer = myRoot.getMainViewer();
      viewer.setOrthographicView (true);
      viewer.setAxialView (AxisAlignedRotation.NY_Z);
      DicomViewer dviewer = myDicomManager.getViewer();
      if (dviewer != null) {
         myDicomManager.setYZVisible (true);
         myDicomManager.setXZVisible (false);
         myDicomManager.setXYVisible (false);
         myRoot.rerender();
      }
   }

   void setCoronalView() {
      Viewer viewer = myRoot.getMainViewer();
      viewer.setOrthographicView (true);
      viewer.setAxialView (AxisAlignedRotation.X_Z);
      DicomViewer dviewer = myDicomManager.getViewer();
      if (dviewer != null) {
         myDicomManager.setYZVisible (false);
         myDicomManager.setXZVisible (true);
         myDicomManager.setXYVisible (false);
         myRoot.rerender();
      }
   }

   void setAxialView() {
      Viewer viewer = myRoot.getMainViewer();
      viewer.setOrthographicView (true);
      viewer.setAxialView (AxisAlignedRotation.X_Y);
      DicomViewer dviewer = myDicomManager.getViewer();
      if (dviewer != null) {
         myDicomManager.setYZVisible (false);
         myDicomManager.setXZVisible (false);
         myDicomManager.setXYVisible (true);
         myRoot.rerender();
      }
   }

   void set3DView() {
      Viewer viewer = myRoot.getMainViewer();
      viewer.setOrthographicView (false);
      DicomViewer dviewer = myDicomManager.getViewer();
      if (dviewer != null) {
         myDicomManager.setYZVisible (true);
         myDicomManager.setXZVisible (true);
         myDicomManager.setXYVisible (true);
         myRoot.rerender();
      }
   }

}
