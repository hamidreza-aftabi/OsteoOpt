package artisynth.VSP.reconstruction;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;

import javax.swing.*;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JSeparator;
import javax.swing.SwingConstants;
import javax.swing.SwingUtilities;
import javax.swing.JTabbedPane;
import javax.swing.event.MenuEvent;
import javax.swing.event.MenuListener;

import artisynth.VSP.reconstruction.MeshManager.MarkerFormat;
import artisynth.VSP.reconstruction.MeshManager.PlaneFormat;
import artisynth.VSP.reconstruction.ReconAppRoot.DonorType;
import artisynth.core.driver.Main;
import artisynth.core.driver.ModelFileChooser;
import artisynth.core.gui.editorManager.FrameMarkerAgent;
import artisynth.core.gui.widgets.MeshFileChooser;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.modelbase.RenderableComponentBase;
import artisynth.core.util.ArtisynthIO;
import artisynth.core.renderables.*;
import maspack.image.dicom.DicomImage;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.io.GenericMeshReader;
import maspack.geometry.io.GenericMeshWriter;
import maspack.geometry.io.MeshWriter;
import maspack.geometry.io.StlWriter;
import maspack.matrix.Plane;
import maspack.matrix.*;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.GenericPropertyHandle;
import maspack.properties.HasProperties;
import maspack.properties.Property;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer.*;
import maspack.util.NumberFormat;
import maspack.widgets.EnumSelector;
import maspack.widgets.GuiUtils;
import maspack.widgets.LabeledComponentBase;
import maspack.widgets.LabeledControl;
import maspack.widgets.LabelSpacing;
import maspack.widgets.PropertyDialog;
import maspack.widgets.PropertyPanel;
import maspack.widgets.PropertyWindow;
import maspack.widgets.ValueChangeEvent;
import maspack.widgets.ValueChangeListener;

/**
 * Base class for GUI tab panels.
 */
public class TabPanel extends JPanel {

   PropertyPanel myPropPanel;
   JPanel myTaskPanel;

   // references to the root model and worker components:
   TaskFrame myTaskFrame;
   MandibleRecon myRoot;

   ArrayList<JButton> myButtons = new ArrayList<>();

   int getMaxButtonWidth() {
      int max = 0;
      for (JButton b : myButtons) {
         Dimension size = b.getPreferredSize();
         if (size.width > max) {
            max = size.width;
         }
      }
      return max;      
   }

   public LabelSpacing getLabelSpacing() {
      LabelSpacing spc = new LabelSpacing();
      myPropPanel.getLabelSpacing(spc);
      return spc;
   }

   public void setLabelSpacing (LabelSpacing spc) {
      myPropPanel.setLabelSpacing (spc);
   }

   void setMaxButtonWidth (int w) {
      for (JButton b : myButtons) {
         Dimension size = b.getPreferredSize();
         size.width = w;
         b.setPreferredSize (size);
      }
   }

   JButton addVerticalButton (
      JPanel panel, String name, String toolTip, ActionListener listener) {
      JButton button = createVerticalButton (panel, name, toolTip, listener);
      myButtons.add (button);
      return button;
   }

   /**
    * Creates a button for inclusion in the task panel.
    */
   JButton createVerticalButton (
      JPanel panel, String name, String toolTip, ActionListener listener) {
      JButton button = new JButton (name);
      button.addActionListener (listener);
      if (toolTip != null) {
         button.setToolTipText (toolTip);
      }
      button.setAlignmentX (Component.LEFT_ALIGNMENT);
      Dimension size = button.getPreferredSize();
      button.setMaximumSize (new Dimension (Short.MAX_VALUE, size.height));
      button.setHorizontalAlignment (SwingConstants.LEFT);
      button.setMargin (new Insets (5, 10, 5, 10));
      panel.add (button);
      panel.add (Box.createRigidArea (new Dimension(0, 2)));
      return button;
   }

   /**
    * Creates the panel containing buttons to perform various tasks for
    * creating and editing reconstruction objects.
    */
   JPanel createTaskPanel () {
      // subclass must implement
      return null;
   }

   protected void addStandardProps (PropertyPanel panel) {
      panel.addWidget (Box.createVerticalGlue());

      panel.addWidget (Box.createRigidArea (new Dimension(0, 10)));
      panel.addWidget (new JSeparator());
      panel.addWidget (myRoot, "renderPlaneWidth");
      panel.addWidget (myRoot, "cutPlaneWidth");
      panel.addWidget (myRoot, "planeResolution");
      panel.addWidget (myRoot, "donorSetBack");
      panel.addWidget (myRoot, "writeStlAsBinary");
      panel.addWidget (myRoot, "use2DBonyContact");
      panel.addWidget (Box.createRigidArea (new Dimension(0, 10)));
   }

   /**
    * Creates a panel that allows editing of various propertues related to
    * reconstruction objects.
    */
   PropertyPanel createPropPanel() {
      // subclass must implement
      return null;
   }

   void initReferences() {
      // subclass must implement      
   }

   void updateButtons() {
      // subclass must implement      
   }

   public void updateWidgetValues () {
      myPropPanel.updateWidgetValues();
      updateButtons();
   }

   /**
    * Connects a set of property widgets, with specified names, to a new
    * property host.
    */
   void updateWidgetsForHost (
      PropertyPanel panel, HasProperties host, String... names) {

      for (String name : names) {
         LabeledComponentBase comp = panel.getPropertyWidget(name);
         String errMsg = null;
         if (comp != null) {
            Property prop = panel.getWidgetProperty (comp);
            if (prop instanceof GenericPropertyHandle) {
               try {
                  ((GenericPropertyHandle)prop).resetHost (host);
               }
               catch (Exception e) {
                  errMsg = e.getMessage();
               }
            }
            else {
               errMsg = "property is not a GenericPropertyHandle";
            }
         }
         else {
            errMsg = "widget not found";
         }
         if (errMsg != null) {
            System.out.println (
               "WARNING: Can't reset host for property '" + name +"': "+errMsg);
         }
      }
   }      

   /**
    * Called whenever a new case is loaded to connect the widgets in the
    * property panel to their (newly created) host components.
    */
   void updatePropPanel() {
      // subclass must implement
   }

   /**
    * Creates a new TaskFrame, consisting of a task panel on the left, property
    * panel on the right, and a menu bar.
    */
   public TabPanel (TaskFrame taskFrame) {
      myTaskFrame = taskFrame;
      myRoot = myTaskFrame.myRoot;

      setLayout (new BorderLayout());

      initReferences();

      myTaskPanel = createTaskPanel();
      add (myTaskPanel, BorderLayout.LINE_START);     

      myPropPanel = createPropPanel();
      add (myPropPanel, BorderLayout.CENTER);     
   }

   /**
    * Wrapper method to rerender the viewer. Called when something
    * related the visualization changes.
    */
   void rerender() {
      myRoot.rerender();
   }

   /**
    * Convenience method to display an error dialog.
    */
   void showError (String msg, Exception e) {
      if (!(e instanceof IOException)) {
         e.printStackTrace(); 
      }
      if (e.getMessage() != null) {
         msg += "\n" + e.getMessage();
      }
      else {
         msg += "\n" + e.toString();
      }
      GuiUtils.showError (this, msg);
   }

   /**
    * Queries whether a component is visible in the viewer.
    */
   boolean isVisible (RenderableComponent r) {
      return RenderableComponentBase.isVisible(r);
   }

   /**
    * Sets the visibilty for a component in the viewer.
    */
   boolean setVisible (RenderableComponent r, boolean enable) {
      return RenderableComponentBase.setVisible(r, enable);
   }

   /**
    * Checks if an item is present, based on {@code present}, and presents and
    * error dialog if it is not.
    */
   boolean checkPresent (boolean present, String itemName) {
      if (!present) {
         GuiUtils.showError (this, itemName + " not present");
         return false;
      }
      else {
         return true;
      }
   }

   protected void enableWidget (
      PropertyPanel panel, String name, boolean enable) {
      Component comp = panel.getWidget(name);
      if (comp instanceof LabeledControl) {
         ((LabeledControl)comp).setEnabledAll (enable);
      }
   }

}
