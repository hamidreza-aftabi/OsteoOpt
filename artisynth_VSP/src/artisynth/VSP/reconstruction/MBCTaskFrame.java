package artisynth.VSP.reconstruction;

import java.awt.*;
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

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JSeparator;
import javax.swing.SwingConstants;
import javax.swing.SwingUtilities;
import javax.swing.event.MenuEvent;
import javax.swing.event.MenuListener;

import artisynth.VSP.reconstruction.MeshManager.MarkerFormat;
import artisynth.VSP.reconstruction.MeshManager.PlaneFormat;
import artisynth.core.driver.Main;
import artisynth.core.driver.ModelFileChooser;
import artisynth.core.gui.editorManager.FrameMarkerAgent;
import artisynth.core.gui.widgets.MeshFileChooser;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponent;
import artisynth.core.modelbase.RenderableComponentBase;
import artisynth.core.util.ArtisynthIO;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.io.GenericMeshReader;
import maspack.geometry.io.GenericMeshWriter;
import maspack.geometry.io.MeshWriter;
import maspack.geometry.io.StlWriter;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.GenericPropertyHandle;
import maspack.properties.HasProperties;
import maspack.properties.Property;
import maspack.collision.*;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer.*;
import maspack.util.NumberFormat;
import maspack.widgets.EnumSelector;
import maspack.widgets.GuiUtils;
import maspack.widgets.LabeledComponentBase;
import maspack.widgets.LabeledControl;
import maspack.widgets.PropertyDialog;
import maspack.widgets.PropertyPanel;
import maspack.widgets.PropertyWindow;
import maspack.widgets.ValueChangeEvent;
import maspack.widgets.ValueChangeListener;

/**
 * GUI window for the MaxillaBonyContact application.
 */
public class MBCTaskFrame extends JFrame
   implements PropertyWindow, HasProperties {

   JPanel myPanel;
   PropertyPanel myPropPanel;
   JPanel myTaskPanel;

   // references to the root model and worker components:
   MaxillaBonyContact myRoot;
   MeshManager myMeshManager;

   // local attributes needed to implement PropertyWindow:
   private Object mySyncObj;
   private LinkedList<ValueChangeListener> myGlobalValueChangeListeners =
      new LinkedList<>();

   JButton myClipMaxillaButton;
   JButton myComputeBonyContactButton;

   PropertyDialog myVisibilityDialog;
   

   // Choosers for various import files. We keep references to these so that we
   // can store previous files and options.
   MeshFileChooser myImportMeshChooser;
   PlaneFileChooser myPlaneFileChooser;

   // Define a set of properties to control visibility of objects. Doing this
   // via properties makes it easy to create the visibilty dialog from a
   // PropertyDialog.

   public static PropertyList myProps =
      new PropertyList (MBCTaskFrame.class);

   static {
      myProps.add (
         "maxillaVisible isMaxillaVisible",
         "make maxilla visible", false);
      myProps.add (
         "donorSegmentVisible isDonorSegmentVisible",
         "make donor visible", false);
      myProps.add (
         "resectionPlanesVisible", 
         "make resection planes visible", false);
      myProps.add (
         "clippedMaxillaVisible isClippedMaxillaVisible",
         "make clipped maxilla visible", false);
      myProps.add (
         "contoursVisible", 
         "make bony contact intersection contours visible", false);
      myProps.add (
         "clipMeshVisible isClipMeshVisible", 
         "make clip mesh visible", false);
      myProps.add (
         "allVisible",
         "make all components visible", false);
   }
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public Property getProperty (String name) {
      return PropertyList.getProperty (name, this);
   }

   // convenience methods creating menu items:

   protected static JMenuItem makeMenuItem (
      String name, String tip, ActionListener listener) {
      JMenuItem item = new JMenuItem(name);
      item.addActionListener(listener);
      item.setToolTipText(tip);
      return item;
   }

   protected static JMenuItem addMenuItem (
      JMenu menu, String label, String tip, ActionListener listener) {
      JMenuItem item = makeMenuItem(label, tip, listener);
      menu.add(item);
      return item;
   }

   /**
    * Dynamically creates the import menu.
    */
   private void createImportMenu (JMenu menu) {
      JMenuItem item;

      item = addMenuItem (
         menu, "Maxilla", "Import maxilla mesh", 
         e->importMaxilla());
      item = addMenuItem (
         menu, "DonorSegment", "Import donor segment mesh",
         e->importDonorSegment());
      item = addMenuItem (
         menu, "Resect planes", "Import resection planes",
         e->importResectionPlanes());
   }

   /**
    * Dynamically creates the show menu.
    */
   private void createShowMenu (JMenu menu) {
      JMenuItem item;

      item = addMenuItem (
         menu, "Visibility dialog",
         "Control visibility of different components",
         e -> showVisibilityDialog());
      
      menu.add (new JSeparator());

      item = addMenuItem (
         menu, "Bony contact",
         "Compute bony contact percentage between donor segments",
         e -> showBonyContact());
      item.setEnabled (computeBonyContactEnabled());
   }         

   /**
    * Menu listener that allows dynamic menu creation. 
    */
   private abstract class MenuListenerBase implements MenuListener {
      public void menuCanceled(MenuEvent m_evt) {
      }

      public void menuDeselected(MenuEvent m_evt) {
         JMenu menu = (JMenu)m_evt.getSource();
         menu.removeAll();
      }
      
      public abstract void menuSelected(MenuEvent m_evt);
   }

   /**
    * Creates the main menu, in which each menu is bound to a function that
    * dynamically creates the menu depending on the current workflow context.
    */
   public void createMenuBar() {
      JMenuBar menuBar = new JMenuBar();
      JMenu menu;

      menu = new JMenu("Import");
      menu.addMenuListener(new MenuListenerBase() {
            public void menuSelected(MenuEvent m_evt) {
               createImportMenu((JMenu)m_evt.getSource());
            }
         });
      menuBar.add (menu);

      menu = new JMenu("Show");
      menu.addMenuListener(new MenuListenerBase() {
            public void menuSelected(MenuEvent m_evt) {
               createShowMenu((JMenu)m_evt.getSource());
            }
         });
      menuBar.add (menu);

      setJMenuBar (menuBar);
   }

   /**
    * Creates a button for inclusion in the task panel.
    */
   private JButton createVerticalButton (
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
      
      JPanel panel = new JPanel();
      panel.setLayout (new BoxLayout (panel, BoxLayout.PAGE_AXIS));

      myClipMaxillaButton = createVerticalButton (
         panel, "Clip maxilla",
         "Clip the maxilla using the resection planes",
         e->clipMaxilla());

      myComputeBonyContactButton = createVerticalButton (
         panel, "Compute bony contact",
         "Compute bony contact between resected maxilla and donor segment",
         e->showBonyContact());

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

      HasProperties host;

      panel.addWidget (myRoot, "renderPlaneWidth");
      panel.addWidget (myRoot, "cutPlaneWidth");
      panel.addWidget (myRoot, "planeResolution");
      panel.addWidget (myRoot, "use2DBonyContact");

      return panel;
   }

   /**
    * Creates a new MBCTaskFrame, consisting of a task panel on the left, property
    * panel on the right, and a menu bar.
    */
   public MBCTaskFrame (String name, MaxillaBonyContact mbc) {
      
      super (name);
      myRoot = mbc;
      initReferences();

      myPanel = new JPanel();
      myPanel.setLayout (new BorderLayout());

      createMenuBar();
      getContentPane().add (myPanel);

      createOrUpdateSubPanels();
      pack();
   }

   void createOrUpdateSubPanels() {

      if (myTaskPanel != null) {
         myPanel.remove (myTaskPanel);
      }
      myTaskPanel = createTaskPanel();
      myPanel.add (myTaskPanel, BorderLayout.LINE_START);     

      if (myPropPanel != null) {
         myPanel.remove (myPropPanel);
      }
      myPropPanel = createPropPanel();
      myPanel.add (myPropPanel, BorderLayout.CENTER);     
   }

   /**
    * Initializes references to worker components from the root model.
    */
   void initReferences() {
      myMeshManager = myRoot.getMeshManager();
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
    * Strips redundant "." parent folders for a file path.
    */
   static File cleanPath (File file) {
      if (file != null) {
         File parent = file.getParentFile();
         if (parent != null && parent.getName().equals(".")) {
            file = new File (parent.getParent(), file.getName());
         }
      }
      return file;
   }

   /**
    * Retrieves the file chooser for importing mesh files, initializing it if
    * necessary.
    */
   private MeshFileChooser getImportMeshChooser() {
      MeshFileChooser chooser = myImportMeshChooser;
      if (chooser == null) {
         chooser = new MeshFileChooser(null, /*forReading=*/true);
         chooser.setCurrentDirectory (myRoot.getWorkingFolder());
         String ext = myMeshManager.getDefaultMeshFileExtension();
         chooser.setFileFilter (chooser.getFilterForExtension (ext));
         myImportMeshChooser = chooser;
      }
      return chooser;
   }

   /**
    * Selects a mesh file for import purposes.
    */
   private File selectImportMeshFile (String name, String approveMsg) {
      MeshFileChooser chooser = getImportMeshChooser();
      File file = myMeshManager.getRigidBodyMeshFile (name);
      if (file != null) {
         chooser.setSelectedFile (file);
      }
      int retval = chooser.showDialog (this, approveMsg);
      if (retval == JFileChooser.APPROVE_OPTION) {
         return cleanPath (chooser.getSelectedFile());
      }
      else {
         return null;
      }
   }      

   /**
    * Imports a polygonal mesh from a file.
    */
   private PolygonalMesh importPolygonalMesh (File file)
      throws IOException {
      MeshBase mesh = GenericMeshReader.readMesh (file);
      if (!(mesh instanceof PolygonalMesh)) {
         throw new IOException ("mesh is not polygonal");
      }
      return (PolygonalMesh)mesh;
   }      

   /**
    * Imports a mesh from a file and stores it as a rigid body with a specified
    * name.
    */
   private void importRigidBody (File file, String name, Vector3d cent)
      throws IOException {
      MeshBase mesh = GenericMeshReader.readMesh (file);
      if (!(mesh instanceof PolygonalMesh)) {
         throw new IOException ("mesh is not polygonal");
      }
      if (cent != null) {
         mesh.computeCentroid (cent);
      }
      myMeshManager.setRigidBody (name, (PolygonalMesh)mesh, file.toString());
      myRoot.getMainViewer ().autoFit ();
      rerender();
   }      

   /**
    * Checks if an item is present, based on {@code present}, and presents and
    * error dialog if it is not.
    */
   private boolean checkPresent (boolean present, String itemName) {
      if (!present) {
         GuiUtils.showError (this, itemName + " not present");
         return false;
      }
      else {
         return true;
      }
   }

   // maxilla mesh

   /**
    * Checks if the maxilla mesh is present.
    */
   boolean hasMaxilla() {
      return myMeshManager.hasMaxilla();
   }

   private boolean checkForMaxilla() {
      return checkPresent (hasMaxilla(), "Maxilla");
   }

   /**
    * Queries whether or not the maxilla is visible.
    */
   public boolean isMaxillaVisible () {
      return isVisible (myMeshManager.getMaxilla());
   }

   /**
    * Sets whether or not the maxilla is visible.
    */
   public void setMaxillaVisible (boolean enable) {
      if (setVisible (myMeshManager.getMaxilla(), enable)) {
         rerender();
      }
   }

   /**
    * Imports the maxilla from a file.
    */
   void importMaxilla (File file) throws IOException {
      PolygonalMesh mesh = importPolygonalMesh (file);
      myMeshManager.setMaxilla (mesh, file.toString());
      clearClippedMaxilla();
      myRoot.getMainViewer ().autoFit ();
      rerender();
   }

   /**
    * Imports the maxilla from a user-selected file.
    */
   private void importMaxilla () {
      File file = selectImportMeshFile ("maxilla", "Import");
      if (file != null) {
         try {
            importMaxilla (file);
            myRoot.setWorkingFolder (myImportMeshChooser.getCurrentDirectory());
         }
         catch (Exception e) {
            showError ("Error reading file "+file, e);
         }
      }
   }

   /**
    * Clears the maxilla and all its dependent components.
    */
   void clearMaxilla () {
      clearClippedMaxilla();
      if (myMeshManager.removeMaxilla ()) {
         rerender();
      }
   }

   // donor segment mesh

   /**
    * Queries whether the donor segment is present.
    */
   boolean hasDonorSegment() {
      return getDonorSegment() != null;
   }

   public RigidBody getDonorSegment() {
      return myMeshManager.getRigidBody ("donorSegment");
   }

   private boolean checkForDonorSegment() {
      return checkPresent (hasDonorSegment(), "Donor segment");
   }

   /**
    * Queries whether or not the donor segment is visible.
    */
   public boolean isDonorSegmentVisible () {
      return isVisible (getDonorSegment());
   }

   /**
    * Sets whether or not the donor segment is visible.
    */
   public void setDonorSegmentVisible (boolean enable) {
      if (setVisible (getDonorSegment(), enable)) {
         rerender();
      }
   }

   public RigidBody setDonorSegment (PolygonalMesh mesh, String fileName) {
      return myMeshManager.setRigidBody ("donorSegment", mesh, null);
   }
      
   /**
    * Imports the donor segment from a file.
    * @param file file containing the donor segment mesh
    * @param autoFit if true, resize the viewing volume
    */
   void importDonorSegment (File file, boolean autoFit) throws IOException {
      PolygonalMesh mesh = importPolygonalMesh (file);
      setDonorSegment (mesh, file.toString());
      if (autoFit) {
         myRoot.getMainViewer ().autoFit ();
      }
      rerender();
   }

   /**
    * Imports the donor segment from a file.
    */
   private void importDonorSegment () {
      File file = selectImportMeshFile ("donorSegment", "Import");
      if (file != null) {
         try {
            importDonorSegment (file, true);
            myRoot.setWorkingFolder (myImportMeshChooser.getCurrentDirectory());
         }
         catch (Exception e) {
            showError ("Error reading file "+file, e);
         }
      }
   }

   /**
    * Removes the donor segment and its dependencies.
    */      
   void clearDonorSegment () {
      if (myMeshManager.removeRigidBody ("donorSegment")) {
         rerender();
      }
   }

   // resection planes

   /**
    * Queries whether the resection planes are present.
    */
   boolean hasResectionPlanes() {
      return myMeshManager.hasResectionPlanes();
   }

   private boolean checkForResectionPlanes() {
      return checkPresent (
         hasResectionPlanes(), "One or both resection planes");
   }

   /**
    * Queries whether or not the resection planes are visible.
    */
   public boolean getResectionPlanesVisible() {
      return myMeshManager.getResectionPlanesVisible();
   }

   /**
    * Sets whether or not the resection planes are visible.
    */
   public void setResectionPlanesVisible (boolean enable) {
      if (myMeshManager.setResectionPlanesVisible (enable)) {
         rerender();
      }
   }

   /**
    * Selects a file for importing or exporting resection plane information.
    */
   private File selectPlaneFile (String approveMsg) {
      PlaneFileChooser chooser = myPlaneFileChooser;
      if (chooser == null) {
         chooser = new PlaneFileChooser(MeshManager.PlaneFormat.ARTISYNTH);
         chooser.setCurrentDirectory (myRoot.getWorkingFolder());
         myPlaneFileChooser = chooser;
      }
      int retval = myPlaneFileChooser.showDialog (this, approveMsg);
      if (retval == JFileChooser.APPROVE_OPTION) {
         return cleanPath (myPlaneFileChooser.getSelectedFile());
      }
      else {
         return null;
      }
   }      

   /**
    * Import the resection planes from a file.
    */
   void importResectionPlanes (
      File file, MeshManager.PlaneFormat format) throws IOException {
      RigidTransform3d[] planes = IOUtils.readResectionPlanes (
         file, format, myMeshManager.getMaxillaCentroid());
      System.out.println ("planes.size=" + planes.length);
      clearClippedMaxilla();
      clearClipMesh();
      myMeshManager.setResectionPlanes (planes);
      rerender();
   }

   /**
    * Import the resection planes from a user-selected file.
    */
   private void importResectionPlanes() {
      File file = selectPlaneFile ("Import");
      if (file != null) {
         try {
            PlaneFileChooser chooser = myPlaneFileChooser;
            importResectionPlanes (file, chooser.getFormat());
            myRoot.setWorkingFolder (chooser.getCurrentDirectory());
         }
         catch (Exception e) {
            showError ("Error reading file "+file, e);
            return;
         }
      }
   }

   /**
    * Create a pair of resection planes in a standard position from which
    * they can be placed the user.
    */
   void createResectionPlanes() {
      clearResectionPlanes();
      myMeshManager.createResectionPlanes();
   }

   /**
    * Removes the resection planes and their dependencies.
    */      
   void clearResectionPlanes() {
      clearClippedMaxilla();
      if (myMeshManager.clearResectionPlanes()) {
         rerender();
      }
   }

   // clipped maxilla mesh
   
   /**
    * Queries if it is possible to create the clipped maxilla.
    */
   private boolean clipMaxillaEnabled() {
      return (hasMaxilla() && hasResectionPlanes());
   }

   /**
    * Queries whether the clipped maxilla is present.
    */
   private boolean hasClippedMaxilla() {
      return ((myMeshManager.getRigidBody ("maxillaL") != null &&
               myMeshManager.getRigidBody ("maxillaR") != null) ||
              (myMeshManager.getRigidBody ("maxillaC") != null));
   }

   private boolean checkForClippedMaxilla() {
      return checkPresent (hasClippedMaxilla(), "Clipped maxilla");
   }

   /**
    * Queries whether or not the clipped maxilla is visible.
    */
   public boolean isClippedMaxillaVisible () {
      if (myMeshManager.getRigidBody ("maxillaC") != null) {
         return isVisible (myMeshManager.getRigidBody ("maxillaC"));
      }
      else {
         return (isVisible (myMeshManager.getRigidBody ("maxillaL")) &&
                 isVisible (myMeshManager.getRigidBody ("maxillaR")));
      }
   }

   /**
    * Sets whether or not the clipped maxilla is visible.
    */
   public void setClippedMaxillaVisible (boolean enable) {
      boolean changed = false;
      if (myMeshManager.getRigidBody ("maxillaC") != null) {
         changed = setVisible (myMeshManager.getRigidBody ("maxillaC"), enable);
      }
      else {
         changed |= setVisible (myMeshManager.getRigidBody ("maxillaL"), enable);
         changed |= setVisible (myMeshManager.getRigidBody ("maxillaR"), enable);
      }
      if (changed) {
         rerender();
      }
   }

   /**
    * Queries whether the clip mesh is present.
    */
   private boolean hasClipMesh() {
      return (myMeshManager.getRigidBody ("clipMesh") != null);

   }

   /**
    * Queries whether or not the clip mesh is visible.
    */
   public boolean isClipMeshVisible () {
      if (myMeshManager.getRigidBody ("clipMesh") != null) {
         return isVisible (myMeshManager.getRigidBody ("clipMesh"));
      }
      else {
         return false;
      }
   }

   /**
    * Sets whether or not the clip mesh is visible.
    */
   public void setClipMeshVisible (boolean enable) {
      boolean changed = false;
      if (myMeshManager.getRigidBody ("clipMesh") != null) {
         changed = setVisible (myMeshManager.getRigidBody ("clipMesh"), enable);
      }
      if (changed) {
         rerender();
      }
   }

   public boolean clearClipMesh() {
      boolean changed = false;
      changed |= myMeshManager.removeRigidBody ("clipMesh");
      if (changed) {
         rerender();
      }
      return changed;
   }

   protected boolean hasContours() {
      ComponentList<MeshComponent> meshes =
         (ComponentList<MeshComponent>)myRoot.getMechModel().get ("meshBodies");
      return meshes.size() > 0;
   }

   /**
    * Queries whether or not the interesection contours are visible.
    */
   public boolean getContoursVisible () {
      ComponentList<MeshComponent> meshes =
         (ComponentList<MeshComponent>)myRoot.getMechModel().get ("meshBodies");
      if (meshes.size() == 0) {
         return false;
      }
      for (MeshComponent mc : meshes) {
         if (!mc.getRenderProps().isVisible()) {
            return false;
         }
      }
      return true;
   }

   /**
    * Sets whether or not the interesection contours are visible.
    */
   public void setContoursVisible (boolean enable) {
      boolean changed = false;
      ComponentList<MeshComponent> meshes =
         (ComponentList<MeshComponent>)myRoot.getMechModel().get ("meshBodies");
      for (MeshComponent mc : meshes) {
         changed |= setVisible (mc, enable);
      }
      if (changed) {
         rerender();
      }
   }

   void createClippedMaxilla () {
      PolygonalMesh maxillaMesh = myMeshManager.getMaxilla().getSurfaceMesh();
      RigidTransform3d[] TPWs = myMeshManager.getResectionPlanes();
      if (TPWs.length == 2) {
         PolygonalMesh clipped = myMeshManager.clipMesh (
            maxillaMesh, TPWs[0]);
         myMeshManager.setRigidBody ("maxillaL", clipped);
         clipped = myMeshManager.clipMesh (
            maxillaMesh, TPWs[1]);
         myMeshManager.setRigidBody ("maxillaR", clipped);
      }
      else {
         // create a frame to create the clip mesh in: origin halfway between
         // the origins of the left and right planes, with z running right to
         // left, and x in the opposite direction of the normal of the top
         // plane.
         RigidTransform3d TMW = new RigidTransform3d();
         TMW.p.add (TPWs[0].p, TPWs[1].p);
         TMW.p.scale (0.5);
         Vector3d zdir = new Vector3d();
         zdir.sub (TPWs[0].p, TPWs[1].p);
         zdir.normalize();
         Vector3d xdir = new Vector3d();
         TPWs[2].R.getColumn (2, xdir);
         xdir.negate();
         TMW.R.setZXDirections (zdir, xdir);
         // create the left, right and top planes and transform them into TMW
         Plane planeL = new Plane(TPWs[0]);
         planeL.inverseTransform (TMW);
         Plane planeR = new Plane(TPWs[1]);
         planeR.inverseTransform (TMW);
         Plane planeT = new Plane(TPWs[2]);
         planeT.inverseTransform (TMW);
         PolygonalMesh clipMesh = 
            ComputeUtils.createThreeSegmentClipMesh (
               new Plane[] { planeL, planeR, planeT },
               60,
               //myRoot.getCutPlaneWidth(),
               myRoot.getPlaneResolution(), 
               /*trimPlaneOnTop=*/true,
               /*fillBack=*/true);
         if (clipMesh != null) {
            clipMesh.transform (TMW);
            SurfaceMeshIntersector smi = new SurfaceMeshIntersector();
            PolygonalMesh clipped = smi.findIntersection (maxillaMesh, clipMesh);
            myMeshManager.setRigidBody ("maxillaC", clipped);
            RigidBody body = myMeshManager.setRigidBody ("clipMesh", clipMesh);
            RenderProps.setFaceColor (body, ReconAppRoot.PLANE_COLOR);
            RenderProps.setBackColor (body, ReconAppRoot.PLANE_BACK_COLOR);
            RenderProps.setFaceStyle (body, FaceStyle.FRONT_AND_BACK);
         }
         else {
            System.out.println ("ERROR: couldn't create clip mesh");
         }
      }
   }

   /**
    * Creates the clipped maxilla.
    */
   void clipMaxilla() {
      if (!checkForMaxilla() || !checkForResectionPlanes()) {
         return;
      }
      try {
         createClippedMaxilla();
      }
      catch (Exception e) {
         showError ("Error clipping maxilla", e);
      }
      setResectionPlanesVisible (false);
      setMaxillaVisible (false);
      rerender();
   }

   /**
    * Removes the clipped maxilla and its dependencies.
    */      
   void clearClippedMaxilla() {
      boolean changed = false;
      changed |= myMeshManager.removeRigidBody ("maxillaL");
      changed |= myMeshManager.removeRigidBody ("maxillaR");
      changed |= myMeshManager.removeRigidBody ("maxillaC");
      if (changed) {
         rerender();
      }
   }

   /**
    * Returns {@code true} if all currently created reconstruction objects are
    * visible.
    */
   public boolean getAllVisible () {
      boolean visible = true;
      visible &= (!hasMaxilla() || isMaxillaVisible());
      visible &= (!hasClippedMaxilla() || isClippedMaxillaVisible());
      visible &= (!hasDonorSegment() || isDonorSegmentVisible());
      visible &= (!hasResectionPlanes() || getResectionPlanesVisible());
      visible &= (!hasResectionPlanes() || getContoursVisible());
      return visible;   
   }

   /**
    * Collectively sets the visibility for all currently created reconstruction
    * objects.
    */
   public void setAllVisible (boolean enable) {
      setMaxillaVisible (enable);
      setClippedMaxillaVisible (enable);
      setDonorSegmentVisible (enable);
      setResectionPlanesVisible (enable);
      setContoursVisible (enable);
   }

   /**
    * List of reconstruction objects whose visibility can be controlled by the
    * visibility dialog.
    */
   private String[] visibilityPropNames = {
      "maxilla",
      "clippedMaxilla",
      "donorSegment",
      "resectionPlanes",
      "contours",
      "clipMesh",
   };

   /**
    * Creates the visibility dialog that controls the visibility of the various
    * reconstruction objects.
    */
   private PropertyDialog createVisibilityDialog () {
      PropertyDialog dialog = new PropertyDialog (
         "Visibility", new PropertyPanel(), "Done");
      for (String name : visibilityPropNames) {
         dialog.addWidget (name, this, name+"Visible");
      }
      dialog.addWidget (new JSeparator());
      dialog.addWidget ("all components", this, "allVisible");
      dialog.pack();
      return dialog;
   }

   /**
    * Updates the a sigle widget in the visibility dialog.
    */
   private void updateVisibiltyWidget (String name, boolean isPresent) {
      if (myVisibilityDialog != null) {
         PropertyPanel panel = myVisibilityDialog.getPropertyPanel();
         ((LabeledControl)panel.getWidget(name)).setEnabledAll (isPresent);
      }
   }

   /**
    * Updates the visibility dialog to disable widgets for components that are
    * not present.
    */
   private void updateVisibilityDialog () {
      updateVisibiltyWidget ("maxilla", hasMaxilla());
      updateVisibiltyWidget ("clippedMaxilla", hasClippedMaxilla());
      updateVisibiltyWidget ("donorSegment", hasDonorSegment());
      updateVisibiltyWidget ("resectionPlanes", hasResectionPlanes());
      updateVisibiltyWidget ("contours", hasContours());
   }

   /**
    * Shows the visibility dialog that controls the visibility of the various
    * reconstruction objects.
    */
   private void showVisibilityDialog () {
      PropertyDialog dialog = myVisibilityDialog; 
      if (dialog == null || !dialog.isDisplayable()) {
         dialog = createVisibilityDialog();
         GuiUtils.locateCenter (dialog, this);
         Main.getMain().registerWindow (dialog);
         myVisibilityDialog = dialog;
         updateVisibilityDialog();
         dialog.setDefaultCloseOperation (PropertyDialog.HIDE_ON_CLOSE);
      }
      dialog.setVisible (true);      
   }

   /**
    * Display the centroid of the original maxillas mesh.
    */
   private void showMaxillaCentroid() {
      if (!hasMaxilla()) {
         GuiUtils.showError (this, "No maxilla loaded");
      }
      else {
         String str = myMeshManager.getMaxillaCentroid().toString ("%9.2f");
         GuiUtils.showResult (this, "Maxilla centroid: [ "+str+" ]");
      }
   }

   /**
    * Queries if it is possible to compute bony contact.
    */
   private boolean computeBonyContactEnabled() {
      return (hasClippedMaxilla() && hasResectionPlanes() && hasDonorSegment());
   }

   private void setContourMesh (String name, PolylineMesh mesh, Color color) {
      FixedMeshBody mbody = myMeshManager.setMeshBody (name, mesh);
      RenderProps.setLineWidth (mbody, 3);
      RenderProps.setLineColor (mbody, color);
   }

   double[] computeBonyContact() {
      PolygonalMesh maxillaL = myMeshManager.getRigidBodyMesh ("maxillaL");
      PolygonalMesh maxillaR = myMeshManager.getRigidBodyMesh ("maxillaR");
      PolygonalMesh donorSeg = myMeshManager.getRigidBodyMesh ("donorSegment");
      RigidTransform3d[] planes = myMeshManager.getResectionPlanes();
      if (maxillaL == null || maxillaR == null ||
          donorSeg == null || planes == null) {
         throw new IllegalStateException (
            "Clipped maxilla, resection planes, or donor segment not present");
      }
      double[] ratios = new double[2];
      RigidTransform3d TPW = new RigidTransform3d();
      TPW.set (planes[0]);
      TPW.mulRotX (Math.PI); // flip normal

      double cpw = myRoot.getCutPlaneWidth();
      int res = myRoot.getPlaneResolution();

      if (myRoot.getUse2DBonyContact()) {
         PolylineMesh cmeshA = new PolylineMesh();
         PolylineMesh cmeshB = new PolylineMesh();
         PolylineMesh cmeshI = new PolylineMesh();

         double[] cr = ComputeUtils.estimateContactRatios2D (
            donorSeg, maxillaL, TPW, cpw, res, cmeshA, cmeshB, cmeshI);
         ratios[0] = cr[0];

         setContourMesh ("intersectL", cmeshI, Color.RED);
         setContourMesh ("contoursAL", cmeshA, Color.GREEN);
         setContourMesh ("contoursBL", cmeshB, Color.BLUE);
      
         cmeshA = new PolylineMesh();
         cmeshB = new PolylineMesh();
         cmeshI = new PolylineMesh();

         TPW.set (planes[1]);
         TPW.mulRotX (Math.PI); // flip normal
      
         cr = ComputeUtils.estimateContactRatios2D (
            donorSeg, maxillaR, TPW, cpw, res, cmeshA, cmeshB, cmeshI);
         ratios[1] = cr[0];

         setContourMesh ("intersectR", cmeshI, Color.RED);
         setContourMesh ("contoursAR", cmeshA, Color.GREEN);
         setContourMesh ("contoursBR", cmeshB, Color.BLUE);
      }
      else {
         PolygonalMesh smeshA = new PolygonalMesh();
         PolygonalMesh smeshI = new PolygonalMesh();

         ratios[0] = ComputeUtils.estimateContactRatio3D (
            donorSeg, maxillaL, TPW, cpw, res, smeshA, smeshI);
         myMeshManager.setMeshBody ("sliceAL", smeshA);
         myMeshManager.setMeshBody ("sliceIL", smeshI);

         smeshA = new PolygonalMesh();
         smeshI = new PolygonalMesh();

         TPW.set (planes[1]);
         TPW.mulRotX (Math.PI); // flip normal
         ratios[1] = ComputeUtils.estimateContactRatio3D (
            donorSeg, maxillaR, TPW, cpw, res, smeshA, smeshI);
      
         myMeshManager.setMeshBody ("sliceAR", smeshA);
         myMeshManager.setMeshBody ("sliceIR", smeshI);
      }
      return ratios;
   }

   String bonyContactString (double[] ratios) {
      NumberFormat fmt = new NumberFormat("%6.4f");
      StringBuilder sb = new StringBuilder();
      sb.append ("Donor segment contact ratios (patient left to right):");
      sb.append ("(");
      sb.append (fmt.format(ratios[0]));
      sb.append ("] ");
      for (int i=1; i<ratios.length-1; i += 2) {
         sb.append ("[");
         sb.append (fmt.format(ratios[i]));
         sb.append (" ");
         sb.append (fmt.format(ratios[i+1]));
         sb.append ("] ");
      }
      sb.append ("[");
      sb.append (fmt.format(ratios[ratios.length-1]));
      sb.append (")");
      return sb.toString();
   }
   
   /**
    * Metric action to compute and display the "bony" contact ratios between
    * adjacent donor segments, or the donor segments and the left and right
    * maxilla resection cuts.
    */
   private void showBonyContact() {
      try {
         double[] ratios = computeBonyContact();
         String str = bonyContactString (ratios);
         System.out.println ("cratios=" + str);
         GuiUtils.showResult (this, str);
      }
      catch (Exception e) {
         showError ("Error computing bony contact", e);
      }
   }

   /**
    * Enables or disables buttons in the task panel depending on the current
    * workflow state.
    */
   public void updateButtons() {
      myClipMaxillaButton.setEnabled (clipMaxillaEnabled());
      myComputeBonyContactButton.setEnabled (computeBonyContactEnabled());
   }

   // Implementation methods for PropertyWindow. This interface allows the
   // MBCTaskFrame to be registered with ArtiSnth's Main class so that widgets
   // will be automatically updated whenever a rerender event occurs.

   /**
    * {@inheritDoc}
    */
   @Override
   public void updateWidgetValues () {
      myPropPanel.updateWidgetValues();
      updateButtons();
      updateVisibilityDialog();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void addGlobalValueChangeListener (ValueChangeListener l) {
      myGlobalValueChangeListeners.add (l);
      myPropPanel.addGlobalValueChangeListener (l);
      // add to propertybearing subcomponents      
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void removeGlobalValueChangeListener (ValueChangeListener l) {
      if (myGlobalValueChangeListeners.remove (l)) {
         myPropPanel.removeGlobalValueChangeListener (l);
         // remove from to propertybearing subcomponents            
      }
   }

   /**
    * {@inheritDoc}
    */  
   @Override
   public ValueChangeListener[] getGlobalValueChangeListeners () {
      return myGlobalValueChangeListeners.toArray (
         new ValueChangeListener[0]);
   }

   void fireGlobalValueChangeListeners (ValueChangeEvent ev) {
      for (ValueChangeListener l : myGlobalValueChangeListeners) {
         l.valueChange (ev);
      }
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public Object getSynchronizeObject () {
      return mySyncObj;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void setSynchronizeObject (Object syncObj) {
      mySyncObj = syncObj;      
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public boolean isLiveUpdatingEnabled () {
      return true;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void dispose() {
      super.dispose();
   }

}
