package artisynth.VSP.reconstruction;

import java.awt.Color;
import java.io.File;
import java.io.PrintWriter;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.RenderableComponent;
import artisynth.core.modelbase.RenderableComponentBase;
import artisynth.core.workspace.RootModel;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.MeshFactory;
import maspack.interpolation.CubicHermiteSpline3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.HasProperties;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.util.Write;

/**
 * Base class for worker components that manage and manipulate the structures
 * in the reconstruction model. Methods are included for accessing and querying
 * components in the reconstructions model's underlying MechModel, and the
 * component also supports the Renderable interface to make it easy for the
 * worker subclasses to implement erendering (which they mainly do for
 * diagnostic purposes).
 */
public class WorkerComponentBase extends RenderableComponentBase {
   
   // Density for model components - needed when doing phyical simulation
   // operations on those components.
   protected double DENSITY = 1e-6;
   protected double INF = Double.POSITIVE_INFINITY;

   MechModel myMech;  // reference to reconstruction model's MechModel

   /**
    * Called after this component is added to the MechModel to initialize any
    * needed references.
    */
   public void initialize() {
      if (getParent() instanceof MechModel) {
         myMech = (MechModel)getParent();
      }
      else {
         throw new IllegalStateException (
            "Component does not have MechModel as a parent");
      }
   }

   public static PropertyList myProps =
      new PropertyList (
         WorkerComponentBase.class, RenderableComponentBase.class);

   static {
      // myProps.add (
      //    "renderProps", "render properties", defaultRenderProps(null));
   }
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public WorkerComponentBase () {
   }

   public ReconAppRoot getRoot() {
      RootModel root = RootModel.getRoot (this);
      if (root instanceof ReconAppRoot) {
         return (ReconAppRoot)root;
      }
      else {
         return null;
      }
   }

   /**
    * Creates a mesh to implement a cut plane for CSG purposes, using the root
    * model's cut plane width and resolution properties.
    */
   PolygonalMesh createCutPlaneMesh () {
      ReconAppRoot root = getRoot();
      double width = root.getCutPlaneWidth();
      int res = root.getPlaneResolution();
      return MeshFactory.createPlane (width, width, res, res);
   }

   // RigidBody accessor methods

   protected RigidBody getRigidBody (String name) {
      return myMech.rigidBodies().get (name);
   }
   
   protected PolygonalMesh getRigidBodyMesh (String name) {
      RigidBody body = getRigidBody (name);
      return body != null ? body.getSurfaceMesh() : null;
   }

   File getRigidBodyMeshFile (String name) {
      RigidBody body = getRigidBody (name);
      if (body != null) {
         String fileName = body.getSurfaceMeshComp().getFileName();
         if (fileName != null) {
            return new File(fileName);
         }
      }
      return null;
   }

   protected void setRigidBody (String name, RigidBody body) {
      removeRigidBody (name);
      body.setName (name);
      body.setDynamic (false);
      myMech.addRigidBody (body);
    }

   protected RigidBody setRigidBody (String name, PolygonalMesh mesh) {
      return setRigidBody (name, mesh, null);
   }

   protected RigidBody setRigidBody (
      String name, PolygonalMesh mesh, String meshFileName) {
      return setRigidBody (name, mesh, meshFileName, meshFileName != null);
   }

   protected RigidBody setRigidBody (
      String name, PolygonalMesh mesh, 
      String meshFileName, boolean centerPose) {
      removeRigidBody (name);
      RigidBody body =
         RigidBody.createFromMesh (name, mesh, meshFileName, DENSITY, 1.0);
      if (centerPose) {
         Point3d cent = new Point3d();
         mesh.computeCentroid (cent);
         body.translateCoordinateFrame (cent);
         body.setPosition (Point3d.ZERO);
      }
      body.setDynamic (false);
      myMech.addRigidBody (body);
      return body;
   }
   
   protected boolean removeRigidBody (String name) {
      RigidBody body = getRigidBody(name);
      if (body != null) {
         myMech.removeRigidBody (body);
         return true;
      }
      else {
         return false;
      }
   } 

   // MeshBody body accessors

   protected FixedMeshBody getMeshBody (String name) {
      return (FixedMeshBody)myMech.meshBodies().get (name);
   }

   protected MeshBase getMeshBodyMesh (String name) {
      FixedMeshBody body = getMeshBody (name);
      return body != null ? body.getMesh() : null;
   }         

   protected FixedMeshBody setMeshBody (String name, MeshBase mesh) {
      removeMeshBody (name);
      FixedMeshBody body = new FixedMeshBody(name, mesh);
      myMech.addMeshBody (body);
      return body;
   }

   protected boolean removeMeshBody (String name) {
      MeshComponent body = getMeshBody(name);
      if (body != null) {
         myMech.removeMeshBody (body);
         return true;
      }
      else {
         return false;
      }
   }

   // spline curve accessor methods

   public Spline3dBody getCurve (String name) {
      RenderableComponent comp = myMech.renderables().get(name);
      if (comp instanceof Spline3dBody) {
         return (Spline3dBody)comp;
      }
      else {
         return null;
      }
   }

   public Spline3dBody setCurve (
      String name, CubicHermiteSpline3d curve, RigidTransform3d TCW) {
      removeCurve(name);
      Spline3dBody body = new Spline3dBody (name, curve);
      body.setPose (TCW);
      RenderProps.setCylindricalLines (body, 1.0, Color.CYAN);
      myMech.addRenderable (body);
      return body;
   }

   public boolean removeCurve (String name) {
      Spline3dBody curve = getCurve (name);
      if (curve != null) {
         myMech.removeRenderable (curve);
         return true;
      }
      else {
         return false;
      }
   }

   // renderable interface

   public void updateBounds (Vector3d pmin, Vector3d pmax) {
   }

   /**
    * Subclasses can override this as needed to provide rendering.
    */
   public void render (Renderer renderer, int flags) {
   }

   // I/O support

   protected void writeFilePath (PrintWriter pw, String name, File file) {
      pw.println (name + Write.getQuotedString(file.toString()));
   }


}
