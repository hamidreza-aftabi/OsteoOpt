package artisynth.VSP.reconstruction;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayDeque;
import java.util.Deque;

import javax.swing.JFrame;
import javax.swing.JMenu;

import artisynth.core.driver.Main;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.AxialSpringList;
import artisynth.core.modelbase.ComponentChangeEvent;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.modelbase.ScanWriteUtils;
import artisynth.core.util.ArtisynthIO;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.ScanToken;
import artisynth.core.workspace.RootModel;
import maspack.geometry.MeshBase;
import maspack.matrix.AxisAlignedRotation;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.properties.PropertyUtils;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.PointStyle;
import maspack.util.IndentingPrintWriter;
import maspack.util.PathFinder;
import maspack.util.ReaderTokenizer;
import maspack.util.Write;

/**
 * Special ArtiSynth application that computes the bony contact for a
 * maxilliary reconstruction involving two resection planes and a single donor
 * segment.
 */
public class MaxillaBonyContact extends ReconAppRoot {

   MBCTaskFrame myTaskFrame; // main GUI winow

   // special containers:
   RenderableComponentList<FixedMeshBody> myResectionPlanes;

   // accessors:
   
   public MBCTaskFrame getTaskFrame() {
      return myTaskFrame;
   }

   private File createFile (File parent, String name) {
      if (parent != null) {
         return new File (parent, name);
      }
      else {
         return new File (name);
      }
   }

   /**
    * Root model build method. Creates the MechModel and populates it with the
    * needed components.
    */
   public void build (String[] args) throws IOException {

      // parse arguments
      File workingDir = null;
      File maxillaFile = null;
      File donorFile = null;
      File planesFile = null;
      for (int i=0;i<args.length; i++) {
         if (args[i].equals ("-workingDir")) {
            if (i == args.length-1) {
               System.out.println (
                  "WARNING: option -workingDir requires another argument");
            }
            workingDir = new File(args[++i]);
         }
         else if (args[i].equals ("-maxilla")) {
            if (i == args.length-1) {
               System.out.println (
                  "WARNING: option -maxilla requires another argument");
            }
            maxillaFile = createFile (workingDir, args[++i]);
         }
         else if (args[i].equals ("-donorSegment")) {
            if (i == args.length-1) {
               System.out.println (
                  "WARNING: option -donorSegment requires another argument");
            }
            donorFile = createFile (workingDir, args[++i]);
         }
         else if (args[i].equals ("-planes")) {
            if (i == args.length-1) {
               System.out.println (
                  "WARNING: option -planes requires another argument");
            }
            planesFile = createFile (workingDir, args[++i]);
         }
         else if (args[i].equals ("-test")) {
            maxillaFile = createFile (workingDir, "MaxillaDec.stl");
            planesFile = createFile (workingDir, "planes2.txt");
         }
         else {
            System.out.println ("WARNING: ignoring unknown option "+args[i]);
            System.out.println ("Options are:");
            System.out.println (" -workingDir <dir> // set working folder");
         }
      }

      myMech = new MechModel ("mech");
      // use static intergration for simulation
      myMech.setIntegrator (Integrator.StaticIncremental);
      myMech.setGravity (0, 0, 0); // turn off gravity
      // default rendering properties for the MechModel:
      RenderProps.setPointStyle (myMech, PointStyle.SPHERE);
      RenderProps.setPointRadius (myMech, 1.25);
      RenderProps.setFaceColor (myMech, BONE_COLOR);
      RenderProps.setSpindleLines (myMech.axialSprings(), 1.25, Color.WHITE);
      addModel (myMech);

      // create special containers

      myResectionPlanes = 
         new RenderableComponentList<FixedMeshBody> (
            FixedMeshBody.class, "resectionPlanes");
      myMech.addFixed (myResectionPlanes);
      RenderProps.setFaceColor (myResectionPlanes, PLANE_COLOR);

      // create worker componets:

      myMeshManager = new MeshManager ("meshManager");
      myMech.addFixed (myMeshManager);
      myMeshManager.initialize();

      // create and place main GUI window:

      myTaskFrame = new MBCTaskFrame ("Maxilla bony contact", this);
      Main.getMain().registerWindow (myTaskFrame);
      myTaskFrame.setVisible(true);
      JFrame frame = getMainFrame();
      if (frame != null) {
         java.awt.Point loc = frame.getLocation();
         myTaskFrame.setLocation (loc.x + frame.getWidth(), loc.y);
      }

      setCutPlaneWidth (300);

      if (workingDir != null) {
         setWorkingFolder (workingDir);
      }
      else {
         setWorkingFolder (ArtisynthPath.getUserHomeFolder());
      }

      if (maxillaFile != null) {
         myTaskFrame.importMaxilla (maxillaFile);
      }
      if (planesFile != null) {
         myTaskFrame.importResectionPlanes (
            planesFile, MeshManager.PlaneFormat.ARTISYNTH);
      }
      if (donorFile != null) {
         myTaskFrame.importDonorSegment (donorFile, false);
      }
      
   }
}
