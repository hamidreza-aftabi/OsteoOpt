package artisynth.VSP.reconstruction;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JSeparator;
import javax.swing.JTabbedPane;
import javax.swing.event.MenuEvent;
import javax.swing.event.MenuListener;

import artisynth.VSP.reconstruction.ReconAppRoot.DonorType;
import artisynth.core.driver.Main;
import artisynth.core.driver.ModelFileChooser;
import artisynth.core.gui.widgets.MeshFileChooser;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponent;
import artisynth.core.modelbase.RenderableComponentBase;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.renderables.DicomViewer;
import artisynth.core.util.ArtisynthIO;
import maspack.geometry.MeshBase;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.io.GenericMeshReader;
import maspack.geometry.io.GenericMeshWriter;
import maspack.geometry.io.MeshWriter;
import maspack.geometry.io.StlWriter;
import maspack.image.dicom.DicomImage;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.properties.GenericPropertyHandle;
import maspack.properties.HasProperties;
import maspack.properties.Property;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
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
 * Primary GUI window for creating mandible reconstruction objects.
 * Provides buttons to initiate operations, widgets for adjusting property
 * values, and menus for saving/loading case files, importing/exporting
 * reconstruction objects, object editing, and showing metrics and
 * more detailed property dialogs.
 */
public class TaskFrame extends JFrame
   implements PropertyWindow, HasProperties {

   JPanel myPanel;
   JTabbedPane myTabbedPane;

   MeshesPanel myMeshesPanel;
   DicomPanel myDicomPanel;
   ImplantsPanel myImplantsPanel;
   SegmentsPanel mySegmentsPanel;
   DonorGuidePanel myDonorGuidePanel;
   MandibleGuidePanel myMandibleGuidePanel;
   PlatePanel myPlatePanel;
   ArrayList<TabPanel> myTabPanels = new ArrayList<>();

   // references to the root model and worker components:
   MandibleRecon myRoot;
   private MeshManager myMeshManager;
   private DicomManager myDicomManager;
   private ImplantsManager myImplantsManager;
   private SegmentGenerator mySegmentGenerator;
   private DonorGuideBuilder myDonorGuideBuilder;
   private MandibleGuideBuilder myMandibleGuideBuilder;
   private PlateBuilder myPlateBuilder;
   private String myCurrentEditDialog;

   // local attributes needed to implement PropertyWindow:
   private Object mySyncObj;
   private LinkedList<ValueChangeListener> myGlobalValueChangeListeners =
      new LinkedList<>();

   // Chooser for selecting the donor type
   private EnumSelector myDonorSelector;

   PropertyDialog myDonorSegDialog;
   PropertyDialog myDonorGuideDialog;
   PropertyDialog myMandibleGuideDialog;
   PropertyDialog myPlateDialog;
   PropertyDialog myVisibilityDialog;

   // Choosers for various import/export files. We keep references to these so
   // that we can store previous files and options.
   private ModelFileChooser myCaseFileChooser;
   private MeshFileChooser myImportMeshChooser;
   private MeshFileChooser myExportMeshChooser;
   private MarkerFileChooser myMarkerFileChooser;
   private PlaneFileChooser myPlaneFileChooser;
   private JFileChooser myExportFolderChooser;
   private ModelFileChooser myDonorTransformFileChooser;
   private JFileChooser myDicomFileChooser;

   // Paths for previously exported files:
   private File myDonorGuideFile;
   private File myMandibleGuideFile;
   private File myReconstructionFile;
   private File myCenteredMandibleFile;
   private File myCenteredMaxillaFile;
   private File myMandibleScrewsFile;
   private File myDonorScrewsFile;
   private File myPlateFemSurfaceFile;
   private File myDonorSegmentsFile;
   private File myDonorComplementFile;
   private File myClippedDonorFile;
   private File myDonorFile;
   private File myResectedMandibleFile;
   private File myResectionFile;
   private File myResectionPlanesFile;
   private File myDonorCutPlanesFile;

   // Define a set of properties to control visibility of objects. Doing this
   // via properties makes it easy to create the visibilty dialog from a
   // PropertyDialog.

   public static PropertyList myProps =
      new PropertyList (TaskFrame.class);

   static {
      myProps.add (
         "mandibleVisible isMandibleVisible",
         "make mandible visible", false);
      myProps.add (
         "expandedMandibleVisible isExpandedMandibleVisible",
         "make expanded mandible visible", false);
      myProps.add (
         "donorVisible isDonorVisible",
         "make donor visible", false);
      myProps.add (
         "maxillaVisible isMaxillaVisible",
         "make maxilla visible", false);
      myProps.add (
         "resectionPlanesVisible", 
         "make resection planes visible", false);
      myProps.add (
         "plateMarkersVisible",
         "make plate markers visible", false);
      myProps.add (
         "angleMarkersVisible",
         "make angle markers visible", false);
      myProps.add (
         "plateCurveVisible isPlateCurveVisible",
         "make plate curve visible", false);
      myProps.add (
         "RDPLineVisible isRDPLineVisible",
         "make RDP line visible", false);
      myProps.add (
         "occlusalPlaneVisible isOcclusalPlaneVisible",
         "make occlusal plane visible", false);
      myProps.add (
         "occlusalSliceVisible isOcclusalSliceVisible",
         "make occlusal slice plane visible", false);
      myProps.add (
         "sagittalSliceVisible isSagittalSliceVisible",
         "make sagittal slice plane visible", false);
      myProps.add (
         "implantsVisible",
         "make implants visible", false);
      myProps.add (
         "clippedMandibleVisible isClippedMandibleVisible",
         "make clipped mandible visible", false);
      myProps.add (
         "clippedDonorVisible isClippedDonorVisible",
         "make clipped donor visible", false);
      myProps.add (
         "donorSegmentsVisible",
         "make donor segments visible", false);
      myProps.add (
         "donorGuideVisible isDonorGuideVisible",
         "make donor guide visible", false);
      myProps.add (
         "mandibleCutBoxesVisible areMandibleCutBoxesVisible",
         "make mandible cut boxes visible", false);
      myProps.add (
         "mandibleFlangesVisible areMandibleFlangesVisible",
         "make mandible flanges visible", false); 
      myProps.add (
         "mandibleHandlePointsVisible",
         "make mandible handle points visible", false); 
      myProps.add (
         "mandibleGuideVisible isMandibleGuideVisible",
         "make mandible guide visible", false);
      myProps.add (
         "reconstructionVisible isReconstructionVisible",
         "make reconstruction guide visible", false);
      myProps.add (
         "plateFemVisible isPlateFemVisible",
         "make plate FEM visible", false);
      myProps.add (
         "plateFemCurveVisible isPlateFemCurveVisible",
         "make plate FEM curve visible", false);
      myProps.add (
         "screwsVisible",
         "make screws visible", false);
      myProps.add (
         "resectionVisible isResectionVisible",
         "make resection visible", false);
      myProps.add (
         "trimPlanesVisible",
         "make trim planes visible", false);
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
    * Dynamically creates the file menu    */
   private void createFileMenu (JMenu menu) {
      JMenuItem item;

      item = addMenuItem (
         menu, "Load case ...", "Load case from a file", 
         e->loadCase());
      item = addMenuItem (
         menu, "Save case", "Save case to the current case file",
         e->saveCase());
      item.setEnabled (myRoot.getCaseFile() != null);
      item = addMenuItem (
         menu, "Save case as ...", "Save case to a file",
         e->saveCaseAs());
   }

   /**
    * Dynamically creates the import menu.
    */
   private void createImportMenu (JMenu menu) {
      JMenuItem item;

      item = addMenuItem (
         menu, "Mandible", "Import mandible mesh", 
         e->importMandible());
      item = addMenuItem (
         menu, "Maxilla", "Import maxilla mesh", 
         e->importMaxilla());
      item = addMenuItem (
         menu, "Donor", "Import donor mesh",
         e->importDonor());
      item = addMenuItem (
         menu, "Resect planes", "Import resection planes",
         e->importResectionPlanes());
      item = addMenuItem (
         menu, "Mandible markers", "Import mandible markers",
         e->importMandibleMarkers());
      item = addMenuItem (
         menu, "Dicom image", "Import dicom image",
         e->myDicomPanel.importDicomImage());
      item.setEnabled (myMeshManager.hasMandible());
   }

   /**
    * Dynamically creates the export menu.
    */
   private void createExportMenu (JMenu menu) {
      JMenuItem item;

      item = addMenuItem (
         menu, "Donor guide", "Export donor guide mesh",
         e->exportDonorGuide());
      item.setEnabled (hasDonorGuide());

      item = addMenuItem (
         menu, "Mandible cut boxes", "Export mandible cut boxes",
         e->exportMandibleCutBoxes());
      item.setEnabled (hasMandibleCutBoxes());
      
      item = addMenuItem (
         menu, "Mandible guide", "Export mandible guide",
         e->exportMandibleGuide());
      item.setEnabled (hasMandibleGuide());

      item = addMenuItem (
         menu, "Reconstruction", "Export reconstruction mesh",
         e->exportReconstruction());
      item.setEnabled (hasReconstruction());

      item = addMenuItem (
         menu, "Resection planes", "Export resection planes", 
         e->exportResectionPlanes());
      item.setEnabled (hasResectionPlanes());

      item = addMenuItem (
         menu, "Mandible markers", "Export mandible markers",
         e->exportMandibleMarkers());
      item.setEnabled (hasPlateMarkers());

      item = addMenuItem (
         menu, "Centered mandible", "Export centered mandible",
         e->exportCenteredMandible());
      item.setEnabled (hasMandible());

      item = addMenuItem (
         menu, "Centered maxilla", "Export centered maxilla",
         e->exportCenteredMaxilla());
      item.setEnabled (hasMaxilla());

      item = addMenuItem (
         menu, "Mandible screws", 
         "Export composite screw mesh in mandible space",
         e->exportMandibleScrews());         
      item.setEnabled (hasScrews());

      item = addMenuItem (
         menu, "Donor screws", 
         "Export composite screw mesh in donor space",
         e->exportDonorScrews());
      item.setEnabled (hasScrews() && hasDonorSegments());

      item = addMenuItem (
         menu, "Donor segments", "Export composite donor segment meshes",
         e->exportDonorSegments());
      item.setEnabled (hasDonorSegments());

      item = addMenuItem (
         menu, "Donor complement", "Export donor complement for single segment",
         e->exportDonorComplement());
      item.setEnabled (numDonorSegments() == 1);

      if (myRoot.getDonorType() == DonorType.FIBULA) {
         item = addMenuItem (
            menu, "Clipped donor", "Export clipped donor mesh",
            e->exportClippedDonor());
         item.setEnabled (hasClippedDonor());
      }
      else {
         item = addMenuItem (
            menu, "Donor", "Export donor mesh",
            e->exportDonor());
         item.setEnabled (hasDonor());
      }

      item = addMenuItem (
         menu, "Resected mandible", "Export resected mandible mesh",
         e->exportResectedMandible());
      item.setEnabled (hasClippedMandible());

      item = addMenuItem (
         menu, "Resection mesh", "Export resection mesh",
         e->exportResectionMesh());
      item.setEnabled (hasClippedMandible());

      item = addMenuItem (
         menu, "Resection plane meshes", 
         "Export composite resection plane meshes",
         e->exportResectionPlaneMeshes());
      item.setEnabled (hasResectionPlanes());

      item = addMenuItem (
         menu, "Donor cut plane meshes", 
         "Export composite donor cut plane meshes",
         e->exportDonorCutPlaneMeshes());
      item.setEnabled (hasDonorSegments());

      item = addMenuItem (
         menu, "Donor segment transforms", 
         "Export the transforms associated with each donor segment",
         e->exportDonorSegmentTransforms());
      item.setEnabled (hasDonorSegments());

      item = addMenuItem (
         menu, "Plate surface with screw holes", 
         "Export the plate surface mesh with screw holes",
         e->exportPlateSurfaceWithHoles());
      item.setEnabled (hasPlateFem());

      menu.add (new JSeparator());
      item = addMenuItem (
         menu, "All primary meshes",
         "Export all primary meshes at once, using standard names",
         e->exportAllPrimaryMeshes());
      item.setEnabled (
         hasMandible() && hasReconstruction() && hasDonorGuide() &&
         hasMandibleGuide() && hasScrews());
   }

   /**
    * Dynamically creates the edit menu.
    */
   private void createEditMenu (JMenu menu) {
      JMenuItem item;

      item = addMenuItem (
         menu, "Clear plate markers", "Remove any existing plate markers",
         e->clearPlateMarkers());
      item.setEnabled (hasPlateMarkers());

      item = addMenuItem (
         menu, "Clear angle markers", "Remove any existing angle markers",
         e->clearAngleMarkers());
      item.setEnabled (hasAngleMarkers());

      item = addMenuItem (
         menu, "Clear donor markers", "Remove any existing donor markers",
         e->clearDonorMarkers());
      item.setEnabled (hasDonorMarkers());

      item = addMenuItem (
         menu, "Clear occlusal plane", "Remove any existing occlusal plane",
         e->clearOcclusalPlane());
      item.setEnabled (hasOcclusalPlane());

      item = addMenuItem (
         menu, "Clear implants", "Remove any existing implants",
         e->clearImplants());
      item.setEnabled (hasImplants());

      item = addMenuItem (
         menu, "Reset segment poses", 
         "Reset segment poses to their original values",
         e->resetSegmentPoses());
      item.setEnabled (hasDonorSegments());

      item = addMenuItem (
         menu, "Align segments left-right", 
         "Rotationally align donor segments from left to right",
         e->alignSegments (/*leftRight=*/true));
      item.setEnabled (hasDonorSegments());

      item = addMenuItem (
         menu, "Align segments right-left", 
         "Rotationally align donor segments from right to left",
         e->alignSegments (/*leftRight=*/false));
      item.setEnabled (hasDonorSegments());

      item = addMenuItem (
         menu, "Create plate surface with screw holes", 
         "Create a mesh showing the plate screw holes added",
         e->createPlateSurfaceWithHoles ());
      item.setEnabled (hasPlateFem() && hasScrews());

      item = addMenuItem (
         menu, "Test reset mesh", 
         "Test reset mesh",
         e->testResetMesh());

   }

   void testResetMesh() {
      RigidBody body = myMeshManager.getMandible();
      PolygonalMesh mesh = body.getSurfaceMesh().clone();
      mesh.scale (1.1);
      body.setSurfaceMesh (mesh);
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
         menu, "Donor segment properties", 
         "Show properties for generating donor segments",
         e -> myDonorSegDialog = showPropDialog (
            "donor segments", mySegmentGenerator, myDonorSegDialog));

      item = addMenuItem (
         menu, "Donor guide properties",
         "Show properties for the donor guide",
         e -> myDonorGuideDialog = showPropDialog (
            "donor guide", myDonorGuideBuilder, myDonorGuideDialog));

      item = addMenuItem (
         menu, "Mandible guide properties",
         "Show properties for the mandible guide",
         e ->  myMandibleGuideDialog = showPropDialog (
            "mandible guide", myMandibleGuideBuilder,
            myMandibleGuideDialog));

      item = addMenuItem (
         menu, "plate properties",
         "Show properties for the plate and screws",
         e -> myPlateDialog = showPropDialog (
            "plate FEM", myPlateBuilder, myPlateDialog));

      item = addMenuItem (
         menu, "mandible centroid",
         "Show centroid of the original mandible mesh",
         e -> showMandibleCentroid());
      item.setEnabled (hasMandible());

      menu.add (new JSeparator());

      item = addMenuItem (
         menu, "RDP line lengths",
         "Show the lengths of the RDP line segments",
         e->showRDPLineLengths());
      item.setEnabled (hasRDPLine());

      item = addMenuItem (
         menu, "HD95 distance",
         "Compute HD95 distance between the reconstruction and the resection",
         e->showHD95Distance());
      item.setEnabled (hasDonorSegments() && hasClippedMandible());

      item = addMenuItem (
         menu, "Volume overlap",
         "Compute volume overlap between the reconstruction and the resection",
         e -> showVolumeOverlap());
      item.setEnabled (hasDonorSegments() && hasClippedMandible());

      item = addMenuItem (
         menu, "Bony contact",
         "Compute bony contact percentage between donor segments",
         e -> showBonyContact());
      item.setEnabled (hasDonorSegments() && hasClippedMandible());
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

      JMenu menu = new JMenu("File");
      menu.addMenuListener(new MenuListenerBase() {
            public void menuSelected(MenuEvent m_evt) {
               createFileMenu((JMenu)m_evt.getSource());
            }
         });
      menuBar.add (menu);

      menu = new JMenu("Import");
      menu.addMenuListener(new MenuListenerBase() {
            public void menuSelected(MenuEvent m_evt) {
               createImportMenu((JMenu)m_evt.getSource());
            }
         });
      menuBar.add (menu);

      menu = new JMenu("Export");
      menu.addMenuListener(new MenuListenerBase() {
            public void menuSelected(MenuEvent m_evt) {
               createExportMenu((JMenu)m_evt.getSource());
            }
         });
      menuBar.add (menu);

      menu = new JMenu("Edit");
      menu.addMenuListener(new MenuListenerBase() {
            public void menuSelected(MenuEvent m_evt) {
               createEditMenu((JMenu)m_evt.getSource());
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

      if (myRoot.hasSpecialMenu()) {
         menu = new JMenu("Special");
         menu.addMenuListener(new MenuListenerBase() {
               public void menuSelected(MenuEvent m_evt) {
                  myRoot.createSpecialMenu((JMenu)m_evt.getSource());
               }
            });
         menuBar.add (menu);
      }

      setJMenuBar (menuBar);
   }

   /**
    * Creates the panel at the top of the task frame.
    */
   JPanel createTopPanel () {
      JPanel panel = new JPanel();
      panel.setLayout (new BoxLayout (panel, BoxLayout.LINE_AXIS));
      myDonorSelector = 
         new EnumSelector (
            " donor type ", myRoot.getDonorType(), DonorType.values());
      myDonorSelector.addValueChangeListener (
         e -> updateDonorType ()); 
      panel.add (myDonorSelector);            

      // myDonorSelector.setBorder (
      //    BorderFactory.createCompoundBorder (
      //       BorderFactory.createEtchedBorder (),
      //       BorderFactory.createEmptyBorder (2, 2, 2, 2)));

      myDonorSelector.setBorder (BorderFactory.createEmptyBorder (6, 6, 6, 6));
      panel.setBorder (BorderFactory.createEtchedBorder());

      return panel;
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
   void updatePropPanels() {
      for (TabPanel panel : myTabPanels) {
         panel.updatePropPanel();
      }
   }

   /**
    * Creates a new TaskFrameX, consisting of a task panel on the left, property
    * panel on the right, and a menu bar.
    */
   public TaskFrame (String name, MandibleRecon mandRecon) {
      
      super (name);
      myRoot = mandRecon;
      initReferences();

      myPanel = new JPanel();
      myPanel.setLayout (new BorderLayout());

      JPanel topPanel = createTopPanel();
      myPanel.add (topPanel, BorderLayout.PAGE_START);

      createMenuBar();
      getContentPane().add (myPanel);

      createOrUpdateSubPanels();
      pack();
   }

   void createOrUpdateSubPanels() {

      if (myTabbedPane != null) {
         myPanel.remove (myTabbedPane);
      }
      myTabbedPane = createTabbedPane();
      myPanel.add (myTabbedPane, BorderLayout.CENTER);     

      if (myDonorSelector.getValue() != myRoot.getDonorType()) {
         myDonorSelector.setValue (myRoot.getDonorType());
      }
   }

   void addTab (String name, JTabbedPane pane, TabPanel panel) {
      pane.add (name, panel);
      myTabPanels.add (panel);
   }

   JTabbedPane createTabbedPane() {
      JTabbedPane pane = new JTabbedPane();

      myTabPanels.clear();

      myMeshesPanel = new MeshesPanel (this);
      addTab ("Meshes", pane, myMeshesPanel);

      myDicomPanel = new DicomPanel (this);
      addTab ("Dicom", pane, myDicomPanel);

      myImplantsPanel = new ImplantsPanel (this);
      addTab ("Implants", pane, myImplantsPanel);

      mySegmentsPanel = new SegmentsPanel (this);
      addTab ("Segments", pane, mySegmentsPanel);

      myDonorGuidePanel = new DonorGuidePanel (this);
      addTab ("DonorGuide", pane, myDonorGuidePanel);

      myMandibleGuidePanel = new MandibleGuidePanel (this);
      addTab ("MandibleGuide", pane, myMandibleGuidePanel);

      myPlatePanel = new PlatePanel (this);
      addTab ("Plate", pane, myPlatePanel);

      // make button widths and label spacings uniform:
      LabelSpacing spc = new LabelSpacing();
      int maxButtonWidth = 0;
      for (TabPanel panel : myTabPanels) {
         int maxw = panel.getMaxButtonWidth();
         if (maxw > maxButtonWidth) {
            maxButtonWidth = maxw;
         }
         spc.expand (panel.getLabelSpacing());
      }
      for (TabPanel panel : myTabPanels) {
         panel.setMaxButtonWidth (maxButtonWidth);
         panel.setLabelSpacing (spc);
      }

      return pane;
   }

   void updateDonorType() {
      DonorType donorType = (DonorType)myDonorSelector.getValue();
      if (donorType != myRoot.getDonorType()) {
         myRoot.setDonorType (donorType);
         clearDonor();
         createOrUpdateSubPanels();
         updateButtons();
         pack();
      }
   }

   /**
    * Initializes references to worker components from the root model.
    */
   void initReferences() {
      myMeshManager = myRoot.getMeshManager();
      myDicomManager = myRoot.getDicomManager();
      myImplantsManager = myRoot.getImplantsManager();
      mySegmentGenerator = myRoot.getSegmentGenerator();
      myDonorGuideBuilder = myRoot.getDonorGuideBuilder();
      myMandibleGuideBuilder = myRoot.getMandibleGuideBuilder();
      myPlateBuilder = myRoot.getPlateBuilder();
      myCurrentEditDialog = null;

      for (TabPanel panel : myTabPanels) {
         panel.initReferences();
      }
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
    * Select a case file for either importing or exporting a case
    */
   private File selectCaseFile (String approveMsg) {
      ModelFileChooser chooser = myCaseFileChooser;
      if (chooser == null) {
         chooser = new ModelFileChooser(null);
         chooser.setCurrentDirectory (myRoot.getWorkingFolder());
         myCaseFileChooser = chooser;
      }
      if (myRoot.getCaseFile() != null) {
         chooser.setSelectedFile (myRoot.getCaseFile());
      }
      int retval = chooser.showDialog (this, approveMsg);
      if (retval == JFileChooser.APPROVE_OPTION) {
         if (approveMsg.equals ("Save")) {
            // saving the case
            return chooser.getSelectedFileWithExtension();
         }
         else {
            return chooser.getSelectedFile();
         }
      }
      else {
         return null;
      }
   }

   /**
    * Select the export folder. Used by {@link #exportAllPrimaryMeshes}.
    */
   private File selectExportFolder() {
      JFileChooser chooser = myExportFolderChooser;
      if (chooser == null) {
         chooser = new JFileChooser();
         chooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
         myExportFolderChooser = chooser;
      }
      chooser.setSelectedFile (myRoot.getExportFolder());      
      int retval = chooser.showDialog (this, "Select");
      if (retval == JFileChooser.APPROVE_OPTION) {
         return chooser.getSelectedFile();
      }
      else {
         return null;
      }
   }

   /**
    * Save a case to the current case file, if it exists.
    */
   private void saveCase() {
      File file = myRoot.getCaseFile();
      if (file != null) {
         try {
            myRoot.saveCase (file);
         }
         catch (Exception e) {
            showError ("Error writing file "+file, e);
            myRoot.setCaseFile (null);
            return;
         }
      }
   }

   /**
    * Save a case to a newly selected case file.
    */
   private void saveCaseAs() {
      File file = selectCaseFile ("Save");
      if (file != null) {
         try {
            if (file.exists() && !GuiUtils.confirmOverwrite (this, file)) {
               return;
            }
            myRoot.setWorkingFolder (myCaseFileChooser.getCurrentDirectory());
            myRoot.saveCase (file);
            myRoot.setCaseFile (file);
         }
         catch (Exception e) {
            showError ("Error writing file "+file, e);
            return;
         }
      }
   }

   /**
    * Load a case from a selected case file.
    */
   private void loadCase() {
      File file = selectCaseFile ("Load");
      if (file != null) {
         try {
            myRoot.loadCase (file);
            myRoot.getMainViewer ().autoFit ();
            rerender();
         }
         catch (Exception e) {
            showError ("Error reading file "+file, e);
            return;
         }
      }
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
    * Retrieves the file chooser for exporting mesh files, initializing it if
    * necessary.
    */
   private MeshFileChooser getExportMeshChooser() {
      MeshFileChooser chooser = myExportMeshChooser;
      if (chooser == null) {
         chooser = new MeshFileChooser(null, /*forReading=*/false);
         chooser.setCurrentDirectory (myRoot.getExportFolder());
         String ext = myMeshManager.getDefaultMeshFileExtension();
         chooser.setFileFilter (chooser.getFilterForExtension (ext));
         myExportMeshChooser = chooser;
      }
      return chooser;
   }

   /**
    * Selects a mesh file for export purposes.
    */
   private File selectExportMeshFile (
      String name, File currentFile, String approveMsg) {
      MeshFileChooser chooser = getExportMeshChooser();
      if (currentFile == null) {
         chooser.setSelectedFile (new File(name));
      }
      else {
         chooser.setSelectedFile (currentFile);
      }
      int retval = chooser.showDialog (this, approveMsg);
      if (retval == JFileChooser.APPROVE_OPTION) {
         return cleanPath (chooser.getSelectedFileWithExtension());
      }
      else {
         return null;
      }
   } 

   public void writeMesh (File file, MeshBase mesh) throws IOException {
      if (!mesh.getMeshToWorld().isIdentity()) {
         mesh = mesh.clone();
         mesh.transform (mesh.getMeshToWorld());
      }
      MeshWriter writer = GenericMeshWriter.createWriter (file);
      if (writer == null) {
         throw new IOException ("Mesh file "+file+" has unrecognized extension");
      }
      if (writer instanceof StlWriter && myRoot.getWriteStlAsBinary()) {
         ((StlWriter)writer).setBinary (true);
      }
      writer.writeMesh (mesh);
   }

   /**
    * Exports a mesh to a specified file.
    */
   public File exportMesh (
      String defaultName, File currentFile, MeshBase mesh) {
      File file = selectExportMeshFile (defaultName, currentFile, "Export");
      if (file != null) {
         try {
            if (!file.exists() || GuiUtils.confirmOverwrite (this, file)) {
               writeMesh (file, mesh);
               myRoot.setExportFolder(
                  myExportMeshChooser.getCurrentDirectory());
               return file;
            }
         }
         catch (Exception e) {
            showError ("Error writing file "+file, e);
         }
      }
      return currentFile;
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

   // mandible mesh

   /**
    * Checks if the mandible mesh is present.
    */
   boolean hasMandible() {
      return myMeshManager.hasMandible();
   }

   boolean checkForMandible() {
      return checkPresent (hasMandible(), "Mandible");
   }

   /**
    * Queries whether or not the mandible is visible.
    */
   public boolean isMandibleVisible () {
      return isVisible (myMeshManager.getMandible());
   }

   /**
    * Sets whether or not the mandible is visible.
    */
   public void setMandibleVisible (boolean enable) {
      if (setVisible (myMeshManager.getMandible(), enable)) {
         rerender();
      }
   }

   /**
    * Imports the mandible from a file.
    */
   public void importMandible (File file) throws IOException {
      PolygonalMesh mesh = importPolygonalMesh (file);
      myMeshManager.setMandible (mesh, file.toString());
      clearPlateMarkers();
      clearClippedMandible();
      myRoot.getMainViewer ().autoFit ();
      rerender();
   }

   /**
    * Imports the mandible from a user-selected file.
    */
   private void importMandible () {
      File file = selectImportMeshFile ("mandible", "Import");
      if (file != null) {
         try {
            importMandible (file);
            myRoot.setWorkingFolder (myImportMeshChooser.getCurrentDirectory());
         }
         catch (Exception e) {
            showError ("Error reading file "+file, e);
         }
      }
   }

   /**
    * Clears the mandible and all its dependent components.
    */
   void clearMandible () {
      clearPlateMarkers();
      clearClippedMandible();
      if (myMeshManager.removeMandible ()) {
         rerender();
      }
   }

   /**
    * Exports the (centered) mandible to a user-selected file.
    */      
   private void exportCenteredMandible() {
      myCenteredMandibleFile = exportMesh (
         "centeredMandible", myCenteredMandibleFile,
         myMeshManager.getMandibleMesh());
   }


   // expanded mandible mesh, used for mandible cutting guides

   /**
    * Checks if the expanded mandible mesh is present.
    */
   boolean hasExpandedMandible() {
      return myMeshManager.hasExpandedMandible();
   }

   /**
    * Queries whether or not the expanded mandible is visible.
    */
   public boolean isExpandedMandibleVisible () {
      return isVisible (myMeshManager.getExpandedMandible());
   }

   /**
    * Sets whether or not the expanded mandible is visible.
    */
   public void setExpandedMandibleVisible (boolean enable) {
      if (setVisible (myMeshManager.getExpandedMandible(), enable)) {
         rerender();
      }
   }

   // maxilla mesh

   /**
    * Checks if the maxilla mesh is present.
    */
   boolean hasMaxilla() {
      return myMeshManager.hasMaxilla();
   }

   boolean checkForMaxilla() {
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
      RigidBody maxilla = myMeshManager.setMaxilla (mesh, file.toString(), false);
      Point3d cent = myMeshManager.getMandibleCentroid();
      if (cent != null) {
         cent = new Point3d(cent);
         maxilla.translateCoordinateFrame (cent);
         maxilla.setPose (new RigidTransform3d());
      }
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
      if (myMeshManager.removeMaxilla ()) {
         rerender();
      }
   }

   /**
    * Exports the (centered) maxilla to a user-selected file.
    */      
   private void exportCenteredMaxilla() {
      myCenteredMaxillaFile = exportMesh (
         "centeredMaxilla", myCenteredMaxillaFile,
         myMeshManager.getMaxillaMesh());
   }

   // donor mesh

   /**
    * Queries whether the donor is present.
    */
   boolean hasDonor() {
      return myMeshManager.hasDonor();
   }

   boolean checkForDonor() {
      return checkPresent (hasDonor(), "Donor");
   }

   /**
    * Queries whether or not the donor is visible.
    */
   public boolean isDonorVisible () {
      return isVisible (myMeshManager.getDonor());
   }

   /**
    * Sets whether or not the donor is visible.
    */
   public void setDonorVisible (boolean enable) {
      if (setVisible (myMeshManager.getDonor(), enable)) {
         rerender();
      }
   }

   /**
    * Imports the donor from a file.
    * @param file file containing the donor mesh
    * @param autoFit if true, resize the viewing volume
    */
   public void importDonor (File file, boolean autoFit) throws IOException {
      PolygonalMesh mesh = importPolygonalMesh (file);
      myMeshManager.setDonor (mesh, file.toString());
      clearClippedDonor();
      clearDonorMarkers();
      if (autoFit) {
         myRoot.getMainViewer ().autoFit ();
      }
      rerender();
   }

   /**
    * Imports the donor from a file.
    */
   private void importDonor () {
      File file = selectImportMeshFile ("donor", "Import");
      if (file != null) {
         try {
            importDonor (file, true);
            myRoot.setWorkingFolder (myImportMeshChooser.getCurrentDirectory());
         }
         catch (Exception e) {
            showError ("Error reading file "+file, e);
         }
      }
   }

   /**
    * Removes the donor and its dependencies.
    */      
   void clearDonor () {
      clearClippedDonor();
      if (myMeshManager.removeDonor()) {
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

   boolean checkForResectionPlanes() {
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
  public void importResectionPlanes (
      File file, MeshManager.PlaneFormat format) throws IOException {
      RigidTransform3d[] planes = IOUtils.readResectionPlanes (
         file, format, myMeshManager.getMandibleCentroid());
      clearClippedMandible();
      clearDonorSegments();
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
    * Removes the resection planes and their dependencies.
    */      
   void clearResectionPlanes() {
      clearRDPLine();
      clearClippedMandible();
      if (myMeshManager.clearResectionPlanes()) {
         rerender();
      }
   }

   /**
    * Exports the resection planes to a file.
    */      
   void exportResectionPlanes (File file, MeshManager.PlaneFormat format) 
      throws IOException {
      RigidTransform3d[] planes = myMeshManager.getResectionPlanes();
      IOUtils.writePlanes (
         file, planes, format, myMeshManager.getMandibleCentroid());
   }

   /**
    * Exports the resection planes to a user-selected file.
    */      
   private void exportResectionPlanes() {
      File file = selectPlaneFile ("Export");
      if (file != null) {
         try {
            if (file.exists() && !GuiUtils.confirmOverwrite (this, file)) {
               return;
            }
            PlaneFileChooser chooser = myPlaneFileChooser;
            exportResectionPlanes (file, chooser.getFormat());
            myRoot.setWorkingFolder (chooser.getCurrentDirectory());
         }
         catch (Exception e) {
            showError ("Error reading file "+file, e);
            return;
         }
      }
   }

   /**
    * Exports the resection plane meshes to a user-selected file.
    */      
   private void exportResectionPlaneMeshes() {
      PolygonalMesh mesh = new PolygonalMesh();
      for (FixedMeshBody pbody : myMeshManager.getResectionPlaneBodies()) {
         mesh.addMesh ((PolygonalMesh)pbody.getMesh(),/*respectTransforms=*/true);
      }
      myResectionPlanesFile = exportMesh (
         "resectionPlanes", myResectionPlanesFile, mesh);
   }

   // import/export of mandible and plate markers

   /**
    * Selects a file for importing or exporting plate and angle markers.
    */
   private File selectMarkerFile (String approveMsg) {
      MarkerFileChooser chooser = myMarkerFileChooser;
      if (chooser == null) {
         chooser =
            new MarkerFileChooser(MeshManager.MarkerFormat.ARTISYNTH, true);
         chooser.setCurrentDirectory (myRoot.getWorkingFolder());
         myMarkerFileChooser = chooser;
      }
      int retval = myMarkerFileChooser.showDialog (this, approveMsg);
      if (retval == JFileChooser.APPROVE_OPTION) {
         return cleanPath (myMarkerFileChooser.getSelectedFile());
      }
      else {
         return null;
      }
   }      

   /**
    * Imports plate markers (and possibly angle markers) from a file.
    */
   public void importMandibleMarkers (
      File file, MeshManager.MarkerFormat format, boolean containsMarkers)
      throws IOException {
      ArrayList<Point3d> points = null;
      points = IOUtils.readMarkerPoints (
         file, format, myMeshManager.getMandibleCentroid());
      clearPlateMarkers();
      int nump = points.size();
      if (containsMarkers) {
         myMeshManager.clearAngleMarkers();
         nump -= 6;
      }
      for (int i=0; i<nump; i++) {
         myPlateBuilder.addPlateMarker (points.get(i));
      }
      setPlateMarkersVisible (true);
      if (nump < points.size()) {
         for (int i=nump; i<points.size(); i++) {
            myMeshManager.addAngleMarker (points.get(i));
         }
         setAngleMarkersVisible (true);
      }
      clearPlateCurve();
      rerender();
   }
   
   /**
    * Imports plate markers (and possibly angle markers) from a
    * user-selected file.
    */
   private void importMandibleMarkers() {
      File file = selectMarkerFile ("Import");
      if (file != null) {
         try {
            MarkerFileChooser chooser = myMarkerFileChooser;
            importMandibleMarkers (
               file, chooser.getFormat(), chooser.getContainsAngleMarkers()); 
            myRoot.setWorkingFolder (chooser.getCurrentDirectory());
         }
         catch (Exception e) {
            showError ("Error reading file "+file, e);
            return;
         }
      }
   }

   /**
    * Exports the plate markers (and possibly angle markers) to a file.
    */      
   void exportMandibleMarkers (
      File file, MeshManager.MarkerFormat format, boolean containsMarkers)
      throws IOException {
      ArrayList<Point3d> points = new ArrayList<>();

      for (FrameMarker mkr : myPlateBuilder.getPlateMarkers()) {
         points.add (new Point3d (mkr.getPosition()));
      }
      if (containsMarkers) {
         for (FrameMarker mkr : myMeshManager.getAngleMarkers()) {
            points.add (new Point3d (mkr.getPosition()));
         }
      }
      IOUtils.writeMarkerPoints (
         file, points, format, myMeshManager.getMandibleCentroid());
   }

   /**
    * Exports the plate markers (and possibly angle markers) to a
    * user-selected file.
    */      
   private void exportMandibleMarkers() {
      File file = selectMarkerFile ("Export");
      if (file != null) {
         try {
            if (file.exists() && !GuiUtils.confirmOverwrite (this, file)) {
               return;
            }
            MarkerFileChooser chooser = myMarkerFileChooser;
            exportMandibleMarkers (
               file, chooser.getFormat(), chooser.getContainsAngleMarkers()); 
            myRoot.setWorkingFolder (chooser.getCurrentDirectory());
         }
         catch (Exception e) {
            showError ("Error reading file "+file, e);
            return;
         }
      }
   }

   void setEditDialog (String str) {
      myCurrentEditDialog = str;
   }

   void requestEditDialogClose() {
      String msg;
      if (myCurrentEditDialog != null) {
         msg = "Close '"+myCurrentEditDialog+"' first";
      }
      else {
         msg = "Close open editing dialog first";
      }
      GuiUtils.showError (this, msg);
   }

   // plate markers

   /**
    * Queries whether plate markers are present.
    */
   boolean hasPlateMarkers() {
      return myPlateBuilder.getPlateMarkers().size() > 0;
   }

   boolean checkForPlateMarkers() { 
      return checkPresent (hasPlateMarkers(), "Plate markers");
   }

   /**
    * Queries whether or not the plate markers are visible.
    */
   public boolean getPlateMarkersVisible() {
      return hasPlateMarkers() && isVisible(myPlateBuilder.getPlateMarkers());
   }

   /**
    * Sets whether or not the plate markers are visible.
    */
   public void setPlateMarkersVisible (boolean enable) {
      if (setVisible (myPlateBuilder.getPlateMarkers(), enable)) {
         rerender();
      }
   }

   /**
    * Removes the plate markers and their dependencies.
    */      
   void clearPlateMarkers() {
      clearPlateCurve();
      if (myPlateBuilder.clearPlateMarkers()) {
         rerender();
      }
   }

   // angle markers

   /**
    * Queries whether angle markers are present.
    */
   boolean hasAngleMarkers() {
      return myMeshManager.getAngleMarkers().size() > 0;
   }

   /**
    * Queries whether or not the angle markers are visible.
    */
   public boolean getAngleMarkersVisible() {
      return (
         hasAngleMarkers() && isVisible(myMeshManager.getAngleMarkers()));
   }

   /**
    * Sets whether or not the angle markers are visible.
    */
   public void setAngleMarkersVisible (boolean enable) {
      if (setVisible (myMeshManager.getAngleMarkers(), enable)) {
         rerender();
      }
   }

   /**
    * Removes the angle markers and their dependencies.
    */      
   void clearAngleMarkers() {
      if (myMeshManager.clearAngleMarkers()) {
         rerender();
      }
   }

   // plate curve

   /**
    * Queries whether the plate curve is present.
    */
   boolean hasPlateCurve() {
      return myPlateBuilder.getPlateCurve() != null;
   }

   boolean checkForPlateCurve() {
      return checkPresent (hasPlateCurve(), "Plate curve");
   }

   /**
    * Queries whether or not the plate curve is visible.
    */
   public boolean isPlateCurveVisible() {
      return isVisible(myPlateBuilder.getPlateCurve());
   }

   /**
    * Sets whether or not the plate curve is visible.
    */
   public void setPlateCurveVisible (boolean enable) {
      if (setVisible (myPlateBuilder.getPlateCurve(), enable)) {
         rerender();
      }
   }

   // /**
   //  * Convenience method used for making videos
   //  */
   // void createPlateCurveAndRDPLine() {
   //    createPlateCurve();
   //    createRDPLine();
   // }

   /**
    * Removes the plate curve and its dependencies.
    */      
   void clearPlateCurve() {
      clearPlateFem();
      clearRDPLine();
      if (myPlateBuilder.removePlateCurve()) {
         rerender();
      }
   }
   
   // RDP line

   /**
    * Queries whether the RDP line is present.
    */
   boolean hasRDPLine() {
      return mySegmentGenerator.hasRDPLine();
   }

   boolean checkForRDPLine() {
      return checkPresent (hasRDPLine(), "RDP line");
   }

   /**
    * Queries whether or not the RDP line is visible.
    */
   public boolean isRDPLineVisible() {
      return mySegmentGenerator.isRDPLineVisible();
   }

   /**
    * Sets whether or not the RDP line is visible.
    */
   public void setRDPLineVisible (boolean enable) {
      if (mySegmentGenerator.setRDPLineVisible(enable)) {
         rerender();
      }
   }

   /**
    * Removes the RDP line and its dependencies.
    */      
   void clearRDPLine() {
      clearDonorSegments();
      if (mySegmentGenerator.clearRDPLine()) {
         rerender();
      }
   }

   // occulsal plane

   /**
    * Queries if it is possible to add the occlusal plane.
    */
   boolean addOcclusalPlaneEnabled() {
      return hasMandible();
   }

   /**
    * Queries whether the occlusal plane is present.
    */
   boolean hasOcclusalPlane() {
      return myMeshManager.hasOcclusalPlane();
   }

   /**
    * Queries whether or not the occlusal plane is visible.
    */
   public boolean isOcclusalPlaneVisible() {
      return (
         hasOcclusalPlane() && isVisible(myMeshManager.getOcclusalPlane()));
   }

   /**
    * Sets whether or not the occlusal plane is visible.
    */
   public void setOcclusalPlaneVisible (boolean enable) {
      if (setVisible (myMeshManager.getOcclusalPlane(), enable)) {
         rerender();
      }
   }

   /**
    * Removes the occlusal plane.
    */      
   void clearOcclusalPlane() {
      if (myMeshManager.clearOcclusalPlane()) {
         rerender();
      }
   }

   // occlusal slice plane

   /**
    * Queries whether the slice plane is present.
    */
   boolean hasOcclusalSlice() {
      return getOcclusalSlice() != null;
   }

   SlicePlane getOcclusalSlice() {
      return (SlicePlane)myRoot.getMechModel().renderables().get (
         "occlusalSlice");
   }

   /**
    * Queries whether or not the slice plane is visible.
    */
   public boolean isOcclusalSliceVisible() {
      return (
         hasOcclusalSlice() && isVisible(getOcclusalSlice()));
   }

   /**
    * Sets whether or not the slice plane is visible.
    */
   public void setOcclusalSliceVisible (boolean enable) {
      if (setVisible (getOcclusalSlice(), enable)) {
         rerender();
      }
   }

   void createOcclusalSlice (RigidTransform3d TSW) {
      clearOcclusalSlice();
      SlicePlane splane = myImplantsManager.createOcclusalSlice (TSW);
      myRoot.getMechModel().addRenderable(splane);
   }

   /**
    * Removes the occlusal plane.
    */      
   void clearOcclusalSlice() {
      if (hasOcclusalSlice()) {
         myRoot.getMechModel().removeRenderable (getOcclusalSlice());
         rerender();
      }
   }

   // sagittal slice plane

   /**
    * Queries whether the slice plane is present.
    */
   boolean hasSagittalSlice() {
      return getSagittalSlice() != null;
   }

   SlicePlane getSagittalSlice() {
      return (SlicePlane)myRoot.getMechModel().renderables().get (
         "sagittalSlice");
   }

   /**
    * Queries whether or not the slice plane is visible.
    */
   public boolean isSagittalSliceVisible() {
      return (
         hasSagittalSlice() && isVisible(getSagittalSlice()));
   }

   /**
    * Sets whether or not the slice plane is visible.
    */
   public void setSagittalSliceVisible (boolean enable) {
      if (setVisible (getSagittalSlice(), enable)) {
         rerender();
      }
   }

   void createSagittalSlice (RigidTransform3d TSW) {
      clearSagittalSlice();
      SlicePlane splane = myImplantsManager.createSagittalSlice (TSW);
      myRoot.getMechModel().addRenderable(splane);
   }

   /**
    * Removes the sagittal plane.
    */      
   void clearSagittalSlice() {
      if (hasSagittalSlice()) {
         myRoot.getMechModel().removeRenderable (getSagittalSlice());
         rerender();
      }
   }

   // implants

   /**
    * Queries whether implants are present.
    */
   boolean hasImplants() {
      return myMeshManager.getImplants().size() > 0;
   }

   /**
    * Queries whether or not the implants are visible.
    */
   public boolean getImplantsVisible() {
      return (
         hasImplants() && isVisible(myMeshManager.getImplants()));
   }

   /**
    * Sets whether or not the implants are visible.
    */
   public void setImplantsVisible (boolean enable) {
      if (setVisible (myMeshManager.getImplants(), enable)) {
         rerender();
      }
   }

   /**
    * Removes the angle markers and their dependencies.
    */      
   void clearImplants() {
      if (myMeshManager.clearImplants()) {
         rerender();
      }
   }

   // clipped mandible mesh
   
   /**
    * Queries whether the clipped mandible is present.
    */
   boolean hasClippedMandible() {
      return myMeshManager.hasClippedMandible();
   }

   boolean checkForClippedMandible() {
      return checkPresent (hasClippedMandible(), "Clipped mandible");
   }

   /**
    * Queries whether or not the clipped mandible is visible.
    */
   public boolean isClippedMandibleVisible () {
      return myMeshManager.isClippedMandibleVisible();
   }

   /**
    * Sets whether or not the clipped mandible is visible.
    */
   public void setClippedMandibleVisible (boolean enable) {
      if (myMeshManager.setClippedMandibleVisible(enable)) {
         rerender();
      }
   }

   /**
    * Removes the clipped mandible and its dependencies.
    */      
   void clearClippedMandible() {
      if (myMeshManager.clearClippedMandible()) {
         rerender();
      }
   }

   /**
    * Exports the resected mandible to a user-selected file.
    */      
   public void exportResectedMandible() {
      RigidBody mandibleL = myMeshManager.getRigidBody("mandibleL");
      RigidBody mandibleR = myMeshManager.getRigidBody("mandibleR");
      PolygonalMesh mesh = mandibleL.getSurfaceMesh().clone();
      mesh.addMesh (mandibleR.getSurfaceMesh(), /*respectTransforms=*/true);
      myResectedMandibleFile = exportMesh (
         "resectedMandible", myResectedMandibleFile, mesh);
   }

   /**
    * Exports the resection mesh to a user-selected file.
    */      
   private void exportResectionMesh() {
      RigidBody resection = myMeshManager.getRigidBody("resection");
      PolygonalMesh mesh = resection.getSurfaceMesh().clone();
      myResectionFile = exportMesh ("resectionMesh", myResectionFile, mesh);
   }

   // resection mesh (created when the mandible is clipped)

   /**
    * Queries whether the resection mesh is present.
    */
   public boolean hasResection() {
      return myMeshManager.getRigidBody ("resection") != null;
   }

   /**
    * Queries whether or not the resection mesh is visible.
    */
   public boolean isResectionVisible () {
      return isVisible (myMeshManager.getRigidBody ("resection"));
   }

   /**
    * Sets whether or not the resection mesh is visible.
    */
   public void setResectionVisible (boolean enable) {
      if (setVisible (myMeshManager.getRigidBody ("resection"), enable)) {
         rerender();
      }
   }

   // clipped donor mesh

   /**
    * Queries whether the clipped donor is present.
    */
   boolean hasClippedDonor() {
      return myMeshManager.hasClippedDonor();
   }

   boolean checkForClippedDonor() {
      return checkPresent (hasClippedDonor(), "Clipped donor");
   }

   /**
    * Queries whether or not the clipped donor is visible.
    */
   public boolean isClippedDonorVisible () {
      return myMeshManager.isClippedDonorVisible();
   }

   /**
    * Sets whether or not the clipped donor is visible.
    */
   public void setClippedDonorVisible (boolean enable) {
      if (myMeshManager.setClippedDonorVisible(enable)) {
         rerender();
      }
   }

   /**
    * Removes the clipped donor and its dependencies.
    */      
   void clearClippedDonor() {
      clearDonorMarkers();
      clearDonorSegments();
      if (myMeshManager.clearClippedDonor()) {
         rerender();
      }
   }

   /**
    * Exports the clipped donor to a user-selected file.
    */      
   private void exportClippedDonor() {
      RigidBody clippedDonor = myMeshManager.getRigidBody("clippedDonor");
      PolygonalMesh mesh = clippedDonor.getSurfaceMesh().clone();
      myRoot.removeDonorSetBack (mesh);
      myClippedDonorFile = exportMesh (
         "clippedDonor", myClippedDonorFile, mesh);
   }

   /**
    * Exports the donor to a user-selected file.
    */      
   private void exportDonor() {
      PolygonalMesh mesh = new PolygonalMesh();
      mesh.addMesh (myMeshManager.getDonorMesh(), /*respectTransforms=*/true);
      myRoot.removeDonorSetBack (mesh);
      myDonorFile = exportMesh ("donor", myDonorFile, mesh);
   }

   // donor markers

   /**
    * Queries whether donor markers are present.
    */
   boolean hasDonorMarkers() {
      return myMeshManager.getDonorMarkers().size() > 0;
   }

   /**
    * Queries the number of donor markers.
    */
   int numDonorMarkers() {
      return myMeshManager.getDonorMarkers().size();
   }

   boolean checkForDonorMarkers (int num) { 
      if (numDonorMarkers() < num) {
         GuiUtils.showError (this, "At least "+num+" donor marker(s) required");
         return false;
      }
      else {
         return true;
      }
   }

   /**
    * Removes the donor markers and their dependencies.
    */
   boolean clearDonorMarkers() {
      return myMeshManager.clearDonorMarkers();
   }

   // donor segments

   /**
    * Queries whether the donor segments are present.
    */
   boolean hasDonorSegments() {
      return numDonorSegments() > 0;
   }

   /**
    * Queries the current number of donor segments present.
    */
   private int numDonorSegments() {
      return mySegmentGenerator.getSegments().size();
   }

   boolean checkForDonorSegments() {
      return checkPresent (hasDonorSegments(), "Donor bodies");
   }

   /**
    * Queries whether or not the donor segments are visible.
    */
   public boolean getDonorSegmentsVisible() {
      return mySegmentGenerator.getSegmentsVisible();
   }

   /**
    * Sets whether or not the donor segments are visible.
    */
   public void setDonorSegmentsVisible (boolean visible) {
      if (mySegmentGenerator.setSegmentsVisible(visible)) {
         rerender();
      }
   }

   // /**
   //  * Convenience method used for making videos.
   //  */
   // void findDonorSegmentsAndReconstruction() {
   //    findDonorSegments();
   //    createReconstruction();
   // }

   /**
    * Removes the donor segments and their dependencies.
    */      
   void clearDonorSegments() {
      clearDonorGuide();
      clearReconstruction();
      if (mySegmentGenerator.clearSegments()) {
         mySegmentGenerator.setDonorEndPoint (Point3d.ZERO);
         rerender();
      }
   }

   /**
    * Exports the donor segments to a user-selected file.
    */      
   private void exportDonorSegments() {
      PolygonalMesh mesh = new PolygonalMesh();
      for (DonorSegmentBody seg : mySegmentGenerator.getSegments()) {
         if (myRoot.getDonorType() == DonorType.SCAPULA) {
            PolygonalMesh meshInD = seg.getMesh().clone();
            meshInD.setMeshToWorld (seg.getPoseD());
            mesh.addMesh (meshInD, /*respectTransforms=*/true);
         }
         else {
            mesh.addMesh (seg.getMesh(), /*respectTransforms=*/true);
         }
      }
      myRoot.removeDonorSetBack (mesh);
      myDonorSegmentsFile = exportMesh (
         "donorSegments", myDonorSegmentsFile, mesh);
   }

   /**
    * Exports the donor complement to a user-selected file.
    */      
   private void exportDonorComplement() {
      ComponentList<DonorSegmentBody> segs = mySegmentGenerator.getSegments();
      if (segs.size() == 1) {
         PolygonalMesh mesh = segs.get(0).createComplementMesh();
         if (mesh == null) {
            GuiUtils.showError (this, "Unable to create complement mesh");
         }
         else {
            myRoot.removeDonorSetBack (mesh);
            myDonorComplementFile = exportMesh (
               "donorComplement", myDonorComplementFile, mesh);
         }
      }
   }

   /**
    * Exports the donor cut plane meshes to a user-selected file.
    */      
   private void exportDonorCutPlaneMeshes() {
      PolygonalMesh mesh = new PolygonalMesh();
      for (DonorSegmentBody seg : mySegmentGenerator.getSegments()) {
         for (PolygonalMesh pmesh : seg.getDonorPlaneMeshes()) {
            mesh.addMesh (pmesh, /*respectTransforms=*/true);
         }
      }
      myRoot.removeDonorSetBack (mesh);
      myDonorCutPlanesFile = exportMesh (
         "donorCutPlanes", myDonorCutPlanesFile, mesh);
   }

   /**
    * Exports the donor segment transforms to a file.
    */      
   private void exportDonorSegmentTransforms (File file) throws IOException {
      PrintWriter pw = null;
      try {
         pw = ArtisynthIO.newIndentingPrintWriter (file);
         mySegmentGenerator.writeSegmentTransforms (pw, new NumberFormat("%g"));
      }
      catch (IOException e) {
         throw e;
      }
      finally {
         if (pw != null) {
            pw.close();
         }
      }
   }

   /**
    * Exports the donor cut planes to a user-selected file.
    */      
   private void exportDonorSegmentTransforms() {
      ModelFileChooser chooser = myDonorTransformFileChooser;
      if (chooser == null) {
         chooser = new ModelFileChooser(null);
         chooser.setCurrentDirectory (myRoot.getWorkingFolder());
         myDonorTransformFileChooser = chooser;
      }
      int retval = chooser.showDialog (this, "Export");
      if (retval == JFileChooser.APPROVE_OPTION) {
         File file = chooser.getSelectedFileWithExtension();
         try {
            if (file.exists() && !GuiUtils.confirmOverwrite (this, file)) {
               return;
            }
            myRoot.setWorkingFolder (chooser.getCurrentDirectory());
            exportDonorSegmentTransforms (file);
         }
         catch (Exception e) {
            showError ("Error writing file "+file, e);
            return;
         }
      }
   }

   // donor guide

   /**
    * Queries whether the donor guide is present.
    */
   boolean hasDonorGuide() {
      return myDonorGuideBuilder.hasGuide();
   }

   /**
    * Queries whether or not the donor guide is visible.
    */
   public boolean isDonorGuideVisible () {
      return myDonorGuideBuilder.isGuideVisible();
   }

   /**
    * Sets whether or not the donor guide is visible.
    */
   public void setDonorGuideVisible (boolean enable) {
      if (myDonorGuideBuilder.setGuideVisible (enable)) {
         rerender();
      }
   }

   /**
    * Removes the donor guide and its dependencies.
    */      
   void clearDonorGuide() {
      if (myDonorGuideBuilder.clearGuide()) {
         rerender();
      }
   }

   /**
    * Exports the donor guide to a user-selected file.
    */      
   private void exportDonorGuide () {
      myDonorGuideFile = exportMesh (
         "donorGuide", myDonorGuideFile,
         myDonorGuideBuilder.getGuide().getMesh());
   }

   // mandible cut boxes

   /**
    * Queries whether the mandible guide is present.
    */
   boolean hasMandibleCutBoxes() {
      return myMandibleGuideBuilder.hasCutBoxes();
   }

   /**
    * Queries whether or not the mandible guide is visible.
    */
   public boolean areMandibleCutBoxesVisible () {
      return myMandibleGuideBuilder.areCutBoxesVisible();
   }

   /**
    * Sets whether or not the mandible guide is visible.
    */
   public void setMandibleCutBoxesVisible (boolean enable) {
      if (myMandibleGuideBuilder.setCutBoxesVisible (enable)) {
         rerender();
      }
   }

   /**
    * Removes the mandible guide and its dependencies.
    */      
   void clearMandibleCutBoxes() {
      myMandibleGuideBuilder.clearCutBoxes();
   }

   /**
    * Exports the mandible guide to a user-selected file.
    */      
   private void exportMandibleCutBoxes () {
      myMandibleGuideFile = exportMesh (
         "mandibleGuide", myMandibleGuideFile,
         myMandibleGuideBuilder.getCutBoxesMesh());
   }

   // mandible flanges

   /**
    * Queries whether the mandible guide is present.
    */
   boolean hasMandibleFlanges() {
      return myMandibleGuideBuilder.hasFlanges();
   }

   /**
    * Queries whether or not the mandible guide is visible.
    */
   public boolean areMandibleFlangesVisible () {
      return myMandibleGuideBuilder.areFlangesVisible();
   }

   /**
    * Sets whether or not the mandible guide is visible.
    */
   public void setMandibleFlangesVisible (boolean enable) {
      if (myMandibleGuideBuilder.setFlangesVisible (enable)) {
         rerender();
      }
   }

   /**
    * Removes the mandible guide and its dependencies.
    */      
   void clearMandibleFlanges() {
      myMandibleGuideBuilder.clearFlanges();
   }

   // mandible handle points

   /**
    * Queries whether the mandible guide is present.
    */
   boolean hasMandibleHandlePoints() {
      return myMandibleGuideBuilder.hasHandlePoints();
   }

   /**
    * Queries whether or not the mandible guide is visible.
    */
   public boolean getMandibleHandlePointsVisible () {
      return myMandibleGuideBuilder.getHandlePointsVisible();
   }

   /**
    * Sets whether or not the mandible guide is visible.
    */
   public void setMandibleHandlePointsVisible (boolean enable) {
      if (myMandibleGuideBuilder.setHandlePointsVisible (enable)) {
         rerender();
      }
   }

   // mandible guide

   /**
    * Queries whether the mandible guide is present.
    */
   boolean hasMandibleGuide() {
      return myMandibleGuideBuilder.hasGuide();
   }

   /**
    * Queries whether or not the mandible guide is visible.
    */
   public boolean isMandibleGuideVisible () {
      return myMandibleGuideBuilder.isGuideVisible();
   }

   /**
    * Sets whether or not the mandible guide is visible.
    */
   public void setMandibleGuideVisible (boolean enable) {
      if (myMandibleGuideBuilder.setGuideVisible (enable)) {
         rerender();
      }
   }

   /**
    * Removes the mandible guide and its dependencies.
    */      
   void clearMandibleGuide() {
      myMandibleGuideBuilder.clearGuide();
   }

   /**
    * Exports the mandible guide to a user-selected file.
    */      
   private void exportMandibleGuide () {
      myMandibleGuideFile = exportMesh (
         "mandibleGuide", myMandibleGuideFile,
         myMandibleGuideBuilder.getGuide().getSurfaceMesh());
   }

   // reconstructed mandible mesh

   /**
    * Queries whether the reconstructed mandible has been created.
    */
   boolean hasReconstruction() {
      return myMeshManager.getRigidBody ("reconstruction") != null;
   }

   boolean checkForReconstruction() {
      return checkPresent (hasReconstruction(), "Reconstruction");
   }

   /**
    * Queries whether or not the reconstructed mandible is visible.
    */
   public boolean isReconstructionVisible () {
      return (hasReconstruction() && 
              isVisible(myMeshManager.getRigidBody("reconstruction")));
   }

   /**
    * Sets whether or not the reconstructed mandible is visible.
    */
   public void setReconstructionVisible (boolean enable) {
      if (setVisible (myMeshManager.getRigidBody("reconstruction"), enable)) {
         rerender();
      }
   }

   /**
    * Removes the reconstructed mandible and its dependencies.
    */  
   void clearReconstruction() {
      if (myMeshManager.clearReconstruction()) {
         rerender();
      }
      clearPlateFem();
   }

   /**
    * Exports the reconstruction mesh to a user-selected file.
    */
   private void exportReconstruction() {
      myReconstructionFile = exportMesh (
         "reconstruction", myReconstructionFile,
         myMeshManager.getRigidBody("reconstruction").getSurfaceMesh());
   }

   // plate FEM

   /**
    * Queries if it is possible to create the plate FEM.
    */
   boolean createPlateFemEnabled() {
      return
         hasPlateMarkers() && hasPlateCurve() &&
         hasMandible() && hasReconstruction();
   }

   /**
    * Queries whether the plate FEM has been created.
    */
   boolean hasPlateFem() {
      return myPlateBuilder.hasPlateFem();
   }

   /**
    * Queries whether or not the plate FEM is visible.
    */
   public boolean isPlateFemVisible () {
      return (myPlateBuilder.isPlateFemVisible());
   }

   /**
    * Sets whether or not the plate FEM is visible.
    */
   public void setPlateFemVisible (boolean enable) {
      if (myPlateBuilder.setPlateFemVisible (enable)) {
         rerender();
      }
   }

   /**
    * Queries whether or not the plate FEM curve is visible.
    */
   public boolean isPlateFemCurveVisible () {
      return (myPlateBuilder.isPlateFemCurveVisible());
   }

   /**
    * Sets whether or not the plate FEM curve is visible.
    */
   public void setPlateFemCurveVisible (boolean enable) {
      if (myPlateBuilder.setPlateFemCurveVisible (enable)) {
         rerender();
      }
   }

   /**
    * Removes the plate FEM and its dependencies.
    */      
   void clearPlateFem() {
      clearScrews();
      myPlateBuilder.clearPlateFem();

   }

   // plate screws

   /**
    * Queries whether the plate screws have been created.
    */
   boolean hasScrews() {
      return myPlateBuilder.hasScrews();
   }

   /**
    * Queries whether or not the plate screws are visible.
    */
   public boolean getScrewsVisible () {
      return myPlateBuilder.getScrewsVisible();
   }

   /**
    * Sets whether or not the plate screws are visible.
    */
   public void setScrewsVisible (boolean enable) {
      if (myPlateBuilder.setScrewsVisible (enable)) {
         rerender();
      }
   }

   /**
    * Removes the plate screws.
    */
   void clearScrews() {
      myPlateBuilder.clearScrews();
   }

   /**
    * Exports the plate screws, in mandible space, to a user-selected
    * file.
    */      
   private void exportMandibleScrews() {
      myMandibleScrewsFile = exportMesh (
         "mandibleScrews", myMandibleScrewsFile,
         myPlateBuilder.getMandibleScrewMesh());
   }

   /**
    * Exports the plate screws, in mandible space, to a user-selected file.
    */      
   private void exportDonorScrews() {
      PolygonalMesh mesh = myPlateBuilder.getDonorScrewMesh(
         mySegmentGenerator.getSegments(), /*selectedScrewsOnly=*/false);
      myRoot.removeDonorSetBack (mesh);
      myDonorScrewsFile = exportMesh (
         "donorScrews", myDonorScrewsFile, mesh);
   }

   /**
    * Exports the plate FEM surface with screw holes.
    */
   private void exportPlateSurfaceWithHoles() {
      PolygonalMesh mesh = myPlateBuilder.getPlateFemSurfaceWithHoles();
      myDonorScrewsFile = exportMesh (
         "plateFemSurface", myPlateFemSurfaceFile, mesh);
   }

   /**
    * Creates a plate surface with screw holes.
    */
   private void createPlateSurfaceWithHoles() {
      PolygonalMesh mesh = myPlateBuilder.getPlateFemSurfaceWithHoles();
      FixedMeshBody body =
         myMeshManager.setMeshBody ("plateSurfaceWithHoles", mesh);
      RenderProps.setFaceColor (body, ReconAppRoot.PLATE_COLOR);
   }

   // donor guide holes

   /**
    * Queries if any plate screws are currently selected.
    */
   boolean hasSelectedScrews() {
      return myPlateBuilder.numSelectedScrews() > 0;
   }

   /**
    * Queries whether the donor guide currently has screw holes.
    */
   boolean hasDonorGuideHoles() {
      return myDonorGuideBuilder.hasHoles();
   }

   /**
    * Removes screw holes from the donor guide.
    */
   void clearDonorGuideHoles() {
      myDonorGuideBuilder.clearHoles();
   }

   // trim planes

   boolean hasTrimPlanes() {
      return mySegmentGenerator.numTrimPlanes() > 0;
   }

   /**
    * Queries whether or not the trim planes are visible.
    */
   public boolean getTrimPlanesVisible () {
      return isVisible (mySegmentGenerator.getTrimPlanes());
   }

   /**
    * Sets whether or not the trim planes are visible.
    */
   public void setTrimPlanesVisible (boolean enable) {
      if (setVisible (mySegmentGenerator.getTrimPlanes(), enable)) {
         rerender();
      }
   }

   private class NamedMesh {
      String myName;
      PolygonalMesh myMesh;
      File myFile;
      
      NamedMesh (String name, PolygonalMesh mesh) {
         myName = name;
         myMesh = mesh;
      }
   }

   /**
    * Exports all primary meshes to files with predefined names in a
    * user-specified export folder.
    */      
   private void exportAllPrimaryMeshes() {

      // assemble all the meshes we want to export

      ArrayList<NamedMesh> meshes = new ArrayList<>();
      PolygonalMesh mesh;
      mesh = myMeshManager.getMandibleMesh();
      if (mesh != null) {
         meshes.add (new NamedMesh ("centeredMandible", mesh));
      }
      mesh = myMeshManager.getRigidBodyMesh("reconstruction");
      if (mesh != null) {
         meshes.add (new NamedMesh ("reconstruction", mesh));
      }
      mesh = (PolygonalMesh)myDonorGuideBuilder.getMeshBodyMesh("donorGuide");
      if (mesh != null) {
         meshes.add (new NamedMesh ("donorGuide", mesh));
      }
      mesh = myMandibleGuideBuilder.getCutBoxesMesh();
      if (mesh.numVertices() > 0) {
         meshes.add (new NamedMesh ("mandibleGuide", mesh));
      }
      mesh = myPlateBuilder.getDonorScrewMesh(
         mySegmentGenerator.getSegments(), /*selectedScrewsOnly=*/false);
      if (mesh.numVertices() > 0) {
         meshes.add (new NamedMesh ("donorScrews", mesh));
      }
      mesh = myPlateBuilder.getMandibleScrewMesh();
      if (mesh.numVertices() > 0) {
         meshes.add (new NamedMesh ("mandibleScrews", mesh));
      }
      if (meshes.size() < 6) {
         System.out.println ("Present meshes:");
         for (NamedMesh nmesh : meshes) {
            System.out.println (" "+nmesh.myName);
         }
         GuiUtils.showError (this, "Not all components are present");
         return;
      }

      File folder = selectExportFolder();
      if (folder != null) {
         myRoot.setExportFolder (folder);
         String ext = getExportMeshChooser().getDefaultFileExtension();
         if (ext == null) {
            ext = myMeshManager.getDefaultMeshFileExtension();
         }
         boolean someFilesExist = false;
         for (NamedMesh nmesh : meshes) {
            nmesh.myFile = new File(folder, nmesh.myName+"."+ext);
            if (nmesh.myFile.exists()) {
               someFilesExist = true;
            }
         }
         if (someFilesExist) {
            if (!GuiUtils.confirmAction (
                   this, "One or more files already exist. Overwrite?")) {
               return;
            }
         }
         NamedMesh nmesh = null;
         try {
            for (int i=0; i<meshes.size(); i++) {
               nmesh = meshes.get(i);
               writeMesh (nmesh.myFile, nmesh.myMesh);
            }
         }
         catch (Exception e) {
            showError ("Error writing file "+nmesh.myFile, e);
         }
      }
   }

   /**
    * Shows, and creates if necessary, a property dialog for a specific model
    * component.
    */
   private PropertyDialog showPropDialog (
      String name, ModelComponent comp, PropertyDialog dialog) {
      if (dialog == null) {
         dialog =
            new PropertyDialog (
               "Edit "+name+" properties", comp, "Done");
         GuiUtils.locateCenter (dialog, this);
         Main.getMain().registerWindow (dialog);
      }
      dialog.setVisible (true);
      return dialog;
   }

   /**
    * Returns {@code true} if all currently created reconstruction objects are
    * visible.
    */
   public boolean getAllVisible () {
      boolean visible = true;
      visible &= (!hasMandible() || isMandibleVisible());
      visible &= (!hasDonor() || isDonorVisible());
      visible &= (!hasResectionPlanes() || getResectionPlanesVisible());
      visible &= (!hasPlateMarkers() || getPlateMarkersVisible());
      visible &= (!hasAngleMarkers() || getAngleMarkersVisible());
      visible &= (!hasPlateCurve() || isPlateCurveVisible());
      visible &= (!hasRDPLine() || isRDPLineVisible());
      visible &= (!hasOcclusalPlane() || isOcclusalPlaneVisible());
      visible &= (!hasImplants() || getImplantsVisible());
      visible &= (!hasClippedMandible() || isClippedMandibleVisible());
      visible &= (!hasClippedDonor() || isClippedDonorVisible());
      visible &= (!hasDonorSegments() || getDonorSegmentsVisible());
      visible &= (!hasDonorGuide() || isDonorGuideVisible());
      visible &= (!hasMandibleCutBoxes() || areMandibleCutBoxesVisible());
      visible &= (!hasMandibleFlanges() || areMandibleFlangesVisible());
      visible &= (!hasMandibleHandlePoints() || getMandibleHandlePointsVisible());
      visible &= (!hasMandibleGuide() || isMandibleGuideVisible());
      visible &= (!hasReconstruction() || isReconstructionVisible());
      visible &= (!hasPlateFem() || isPlateFemVisible());
      visible &= (!hasPlateFem() || isPlateFemCurveVisible());
      visible &= (!hasScrews() || getScrewsVisible());
      visible &= (!hasResection() || isResectionVisible());
      visible &= (!hasTrimPlanes() || getTrimPlanesVisible());
      return visible;   
   }

   /**
    * Collectively sets the visibility for all currently created reconstruction
    * objects.
    */
   public void setAllVisible (boolean enable) {
      setMandibleVisible (enable);
      setDonorVisible (enable);
      setResectionPlanesVisible (enable);
      setPlateMarkersVisible (enable);
      setAngleMarkersVisible (enable);
      setPlateCurveVisible (enable);
      setRDPLineVisible (enable);
      setOcclusalPlaneVisible (enable);
      setImplantsVisible (enable);
      setClippedMandibleVisible (enable);
      setClippedDonorVisible (enable);
      setDonorSegmentsVisible (enable);
      setDonorGuideVisible (enable);
      setMandibleCutBoxesVisible (enable);
      setMandibleFlangesVisible (enable);
      setMandibleHandlePointsVisible (enable);
      setMandibleGuideVisible (enable);
      setExpandedMandibleVisible (enable);
      setReconstructionVisible (enable);
      setPlateFemVisible (enable);
      setPlateFemCurveVisible (enable);
      setScrewsVisible (enable);
      setResectionVisible (enable);
      setTrimPlanesVisible (enable);
   }

   /**
    * List of reconstruction objects whose visibility can be controlled by the
    * visibility dialog.
    */
   private String[] visibilityPropNames = {
      "mandible",
      "maxilla",
      "donor",
      "resectionPlanes",
      "plateMarkers",
      "angleMarkers",
      "plateCurve",
      "RDPLine",
      "occlusalPlane",
      "occlusalSlice",
      "sagittalSlice",
      "implants",
      "clippedMandible",
      "clippedDonor",
      "resection",
      "donorSegments",
      "donorGuide",
      "mandibleCutBoxes",
      "mandibleFlanges",      
      "mandibleHandlePoints",      
      "mandibleGuide",
      "expandedMandible",
      "reconstruction",
      "plateFem",
      "plateFemCurve",
      "trimPlanes",
      "screws"
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
      updateVisibiltyWidget ("mandible", hasMandible());
      updateVisibiltyWidget ("maxilla", hasMaxilla());
      updateVisibiltyWidget ("donor", hasDonor());
      updateVisibiltyWidget ("resectionPlanes", hasResectionPlanes());
      updateVisibiltyWidget ("plateMarkers", hasPlateMarkers());
      updateVisibiltyWidget ("angleMarkers", hasAngleMarkers());
      updateVisibiltyWidget ("plateCurve", hasPlateCurve());
      updateVisibiltyWidget ("RDPLine", hasRDPLine());
      updateVisibiltyWidget ("occlusalPlane", hasOcclusalPlane());
      updateVisibiltyWidget ("occlusalSlice", hasOcclusalSlice());
      updateVisibiltyWidget ("sagittalSlice", hasSagittalSlice());
      updateVisibiltyWidget ("implants", hasImplants());
      updateVisibiltyWidget ("clippedMandible", hasClippedMandible());
      updateVisibiltyWidget ("clippedDonor", hasClippedDonor());
      updateVisibiltyWidget ("resection", hasClippedMandible());
      updateVisibiltyWidget ("donorSegments", hasDonorSegments());
      updateVisibiltyWidget ("donorGuide", hasDonorGuide());
      updateVisibiltyWidget ("mandibleCutBoxes", hasMandibleCutBoxes());
      updateVisibiltyWidget ("mandibleFlanges", hasMandibleFlanges());
      updateVisibiltyWidget ("mandibleHandlePoints", hasMandibleHandlePoints());
      updateVisibiltyWidget ("mandibleGuide", hasMandibleGuide());
      updateVisibiltyWidget ("expandedMandible", hasExpandedMandible());
      updateVisibiltyWidget ("reconstruction", hasReconstruction());
      updateVisibiltyWidget ("trimPlanes", hasTrimPlanes());
      updateVisibiltyWidget ("plateFem", hasPlateFem());
      updateVisibiltyWidget ("screws", hasScrews());
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
    * Display the centroid of the original mandibles mesh.
    */
   private void showMandibleCentroid() {
      if (!hasMandible()) {
         GuiUtils.showError (this, "No mandible loaded");
      }
      else {
         String str = myMeshManager.getMandibleCentroid().toString ("%9.2f");
         GuiUtils.showResult (this, "Mandible centroid: [ "+str+" ]");
      }
   }

   /**
    * Display the lengths of the RDP line segments.
    */
   private void showRDPLineLengths() {
      RenderableComponentList<RigidBody> frames =
         mySegmentGenerator.getRDPFrames();
      if (frames == null || frames.size() == 0) {
         GuiUtils.showError (this, "RDP line not present");
         return;
      }
      StringBuilder sb = new StringBuilder();
      sb.append ("RDP line lengths, left to right:");
      NumberFormat fmt = new NumberFormat ("%.6g");
      Point3d lastPnt = null;
      for (int i=0; i<frames.size(); i++) {
         Point3d pnt = frames.get(i).getPosition();
         if (lastPnt != null) {
            sb.append (" " + fmt.format(pnt.distance(lastPnt)));
            if (i < frames.size()-1) {
               sb.append (',');
            }
         }
         lastPnt = pnt;
      }
      System.out.println (sb.toString());
      GuiUtils.showResult (this, sb.toString());
   }

   double computeHD95Distance() {
      Collection<DonorSegmentBody> donorSegs = mySegmentGenerator.getSegments();
      RigidBody resection = myMeshManager.getRigidBody ("resection");
      if (donorSegs == null || resection == null) {
         throw new IllegalStateException (
            "donor segments or resection not present");
      }
      PolygonalMesh combinedSegs =
         myMeshManager.computeReconstruction (null, null, donorSegs);
      return ComputeUtils.get3DHausdorff95 (
         resection.getSurfaceMesh(), combinedSegs);
   }

   /**
    * Metric action to compute and display HD95 distance between the resection
    * mesh and the reconstruction.
    */
   private void showHD95Distance() {
      Collection<DonorSegmentBody> donorSegs = mySegmentGenerator.getSegments();
      RigidBody resection = myMeshManager.getRigidBody ("resection");
      if (resection == null || donorSegs.size() == 0) {
         GuiUtils.showError (
            this, "Resection and/or donor segments not present");
         return;
      }
      try {
         double hd95 = computeHD95Distance();
         System.out.println ("hd95 distance: "+hd95);
         NumberFormat fmt = new NumberFormat ("%.6g");
         GuiUtils.showResult (this, "HD95 distance: "+fmt.format(hd95));
      }
      catch (Exception e) {
         showError ("Error computing HD95 distance", e);
      }
   }

   double computeVolumeOverlap() {
      Collection<DonorSegmentBody> donorSegs = mySegmentGenerator.getSegments();
      RigidBody resection = myMeshManager.getRigidBody ("resection");
      if (donorSegs == null || resection == null) {
         throw new IllegalStateException (
            "donor segments or resection not present");
      }
      PolygonalMesh combinedSegs =
         myMeshManager.computeReconstruction (null, null, donorSegs);
      return ComputeUtils.getVolumeOverlap (
         resection.getSurfaceMesh(), combinedSegs);
   }

   /**
    * Metric action to compute and display the volume overlap between the
    * resection mesh and the reconstruction.
    */
   private void showVolumeOverlap() {
      Collection<DonorSegmentBody> donorSegs = mySegmentGenerator.getSegments();
      RigidBody resection = myMeshManager.getRigidBody ("resection");
      if (resection == null || donorSegs.size() == 0) {
         GuiUtils.showError (
            this, "Resection and/or donor segments not present");
         return;
      }
      try {
         double overlap = computeVolumeOverlap();
         NumberFormat fmt = new NumberFormat("%5.2f");         
         System.out.println ("volume overlap: "+100*overlap+"%");         
         GuiUtils.showResult (
            this, "Volume overlap: "+fmt.format(100*overlap)+"%");
      }
      catch (Exception e) {
         showError ("Error computing volume overlap", e);
      }
   }

   public double[] computeBonyContact() {
      ComponentList<DonorSegmentBody> donorSegs =
         mySegmentGenerator.getSegments();
      PolygonalMesh mandibleL = myMeshManager.getRigidBodyMesh ("mandibleL");
      PolygonalMesh mandibleR = myMeshManager.getRigidBodyMesh ("mandibleR");
      if (mandibleL == null || mandibleR == null || donorSegs.size() == 0) {
         throw new IllegalStateException (
            "Clipped mandible and/or donor segments not present");
      }
      if (myRoot.getUse2DBonyContact()) {
         return ComputeUtils.estimateContactRatios2D (
            donorSegs, mandibleL, mandibleR);
      }
      else {
         return ComputeUtils.estimateContactRatios3D (
            donorSegs, mandibleL, mandibleR);
      }
   }

   private String contactRatioString (double[] ratios) {
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
    * mandible resection cuts.
    */
   private void showBonyContact() {
      try {
         double[] ratios = computeBonyContact();
         String str = contactRatioString (ratios);
         System.out.println ("ratios: " + str);
         GuiUtils.showResult (this, str);
      }
      catch (Exception e) {
         showError ("Error computing bony contact", e);
      }
   }

   /**
    * Closes a property dialog.
    */
   void clearPropDialog (PropertyDialog dialog) {
      if (dialog != null) {
         Main.getMain().deregisterWindow (dialog);
         dialog.dispose();
      }
   }

   /**
    * Clear any open property dialogs. Called whenever a new case file loaded.
    */
   void clearPropDialogs () {
      clearPropDialog (myDonorSegDialog);
      myDonorSegDialog = null;
      clearPropDialog (myDonorGuideDialog);
      myDonorGuideDialog = null;
      clearPropDialog (myMandibleGuideDialog);
      myMandibleGuideDialog = null;
      clearPropDialog (myPlateDialog);
      myPlateDialog = null;
   }

   /**
    * Resets donor segment poses to their initialy computed values.
    */
   void resetSegmentPoses () {
      for (DonorSegmentBody seg : mySegmentGenerator.getSegments()) {
         seg.setPoseD (seg.getPoseD0());
      }
      rerender();
   }

   /**
    * Aligns donor segments to have a consistent orientation with respect to
    * the donor centerline. This helps ensure decent end-on-end contact between
    * adjacent segments.
    */
   void alignSegments (boolean leftToRight) {
      mySegmentGenerator.alignSegments (leftToRight);
      rerender();
   }

   /**
    * Enables or disables buttons in the task panel depending on the current
    * workflow state.
    */
   public void updateButtons() {
      for (TabPanel panel : myTabPanels) {
         panel.updateButtons();
      }
   }

   // Implementation methods for PropertyWindow. This interface allows the
   // TaskFrameX to be registered with ArtiSnth's Main class so that widgets
   // will be automatically updated whenever a rerender event occurs.

   /**
    * {@inheritDoc}
    */
   @Override
   public void updateWidgetValues () {
      for (TabPanel panel : myTabPanels) {
         panel.updateWidgetValues();
      }
      updateVisibilityDialog();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void addGlobalValueChangeListener (ValueChangeListener l) {
      myGlobalValueChangeListeners.add (l);
      for (TabPanel panel : myTabPanels) {
         panel.myPropPanel.addGlobalValueChangeListener (l);
      }
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void removeGlobalValueChangeListener (ValueChangeListener l) {
      if (myGlobalValueChangeListeners.remove (l)) {
         for (TabPanel panel : myTabPanels) {
            panel.myPropPanel.removeGlobalValueChangeListener (l);
         }
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

   /* --- dicom --- */

   boolean hasDicomViewer() {
      MechModel mech = myRoot.getMechModel();
      return getDicomViewer() != null;
   }

   DicomViewer getDicomViewer() {
      MechModel mech = myRoot.getMechModel();
      return (DicomViewer)mech.renderables().get("DicomViewer");   }

}
