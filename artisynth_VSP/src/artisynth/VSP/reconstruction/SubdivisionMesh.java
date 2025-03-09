package artisynth.VSP.reconstruction;

import java.awt.Color;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Deque;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.HasCoordinateFrame;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableCompositeBase;
import artisynth.core.modelbase.TransformGeometryContext;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.core.modelbase.*;
import artisynth.core.workspace.RootModel;
import artisynth.core.renderables.*;
import artisynth.core.util.ScanToken;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.GeometryTransformer;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.AxisAngle;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.HasProperties;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.properties.PropertyUtils;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.*;
import maspack.render.Renderer.AxisDrawStyle;
import maspack.render.Renderer.FaceStyle;
import maspack.util.IndentingPrintWriter;
import maspack.util.FunctionTimer;
import maspack.util.InternalErrorException;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;

/**
 * Custom component representing a subdivision mesh and its editable control
 * points.
 */
public class SubdivisionMesh extends RenderableCompositeBase 
   implements HasCoordinateFrame, TransformableGeometry {

   static private double INF = Double.POSITIVE_INFINITY;
   static private double DTOR = Math.PI/180.0;

   static private final int POSE_FORMAT = RigidTransform3d.MATRIX_3X4_STRING;

   EditablePolygonalMeshComp myLattice;
   FixedMeshBody mySmoothed;

   protected static double DEFAULT_AXIS_LENGTH = 0.0;
   double myAxisLength = DEFAULT_AXIS_LENGTH;

   protected static final AxisDrawStyle DEFAULT_AXIS_RENDER_STYLE =
      AxisDrawStyle.ARROW;
   protected AxisDrawStyle myAxisDrawStyle = DEFAULT_AXIS_RENDER_STYLE;

   public static int DEFAULT_FLAT_DIVISIONS = 1;
   protected int myFlatDivisions = DEFAULT_FLAT_DIVISIONS;

   public static int DEFAULT_SUB_DIVISIONS = 2;
   protected int mySubDivisions = DEFAULT_SUB_DIVISIONS;

   protected static RenderProps defaultRenderProps (HasProperties host) {
      RenderProps props = RenderProps.createRenderProps (host);
      return props;
   }

   public static PropertyList myProps =
      new PropertyList (SubdivisionMesh.class, RenderableCompositeBase.class);

   static {
      myProps.add (
         "renderProps", "render properties", defaultRenderProps(null));
      myProps.add (
         "position", "position of the body coordinate frame",null,"NW");
      myProps.add (
         "orientation", "orientation of the body coordinate frame", null, "NW");
      myProps.add (
         "axisLength", "length of rendered frame axes", DEFAULT_AXIS_LENGTH);
      myProps.add (
         "axisDrawStyle", "rendering style for the frame axes",
         DEFAULT_AXIS_RENDER_STYLE);
      myProps.add (
         "flatDivisions", "number of flat divisions", DEFAULT_FLAT_DIVISIONS);
      myProps.add (
         "subDivisions", "number of subdivisions", DEFAULT_SUB_DIVISIONS);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   // property accessors
   
   public Point3d getPosition() {
      return new Point3d (getPose().p);
   }

   public void setPosition (Point3d pos) {
      RigidTransform3d TMW = new RigidTransform3d (getPose());
      TMW.p.set (pos);
      setPose (TMW);
   }

   public AxisAngle getOrientation() {
      return getPose().R.getAxisAngle();
   }

   public void setOrientation (AxisAngle axisAng) {
      RigidTransform3d TMW = new RigidTransform3d (getPose());
      TMW.R.setAxisAngle (axisAng);
      setPose (TMW);
   }

   public double getAxisLength() {
      return myAxisLength;
   }

   public void setAxisLength (double len) {
      myAxisLength = Math.max (0, len);
   }

   public AxisDrawStyle getAxisDrawStyle() {
      return myAxisDrawStyle;
   }

   public void setAxisDrawStyle (AxisDrawStyle style) {
      myAxisDrawStyle = style;
   }

   public int getSubDivisions() {
      return mySubDivisions;
   }

   public void setSubDivisions (int n) {
      mySubDivisions = n;
      updateSmoothedMesh();
   }

   public int getFlatDivisions() {
      return myFlatDivisions;
   }

   public void setFlatDivisions (int n) {
      myFlatDivisions = n;
      updateSmoothedMesh();
   }

   // other accessors

   public PolygonalMesh getSmoothedMesh() {
      return (PolygonalMesh)mySmoothed.getMesh();
   }

   public EditablePolygonalMeshComp getLattice() {
      return myLattice;
   }

   public VertexComponent getLatticeVertex (int idx) {
      return myLattice.getVertexComponents().get(idx);
   }

   // constructors

   /**
    * Note: need a public no-args constructor for ArtiSynth save/load.
    */
   public SubdivisionMesh () {
      myLattice = new EditablePolygonalMeshComp ();
      myLattice.setName ("lattice");
      myLattice.getFaceList().setSelectable(false);
      add (myLattice);

      mySmoothed = new FixedMeshBody ("smoothed");
      mySmoothed.setSelectable (false);
      add (mySmoothed);

      RenderProps.setSphericalPoints (myLattice, 0.03, Color.CYAN);
      RenderProps.setFaceStyle (myLattice, FaceStyle.NONE);
      RenderProps.setDrawEdges (myLattice, true);
      RenderProps.setEdgeColor (myLattice, Color.BLUE);
      RenderProps.setShading (myLattice.getFaceList(), Shading.NONE);

      //RenderProps.setDrawEdges (mySmoothed, true);
      //RenderProps.setFaceColor (mySmoothed, new Color (0.6f, 0.6f, 1f));
      RenderProps.setEdgeColor (mySmoothed, Color.BLUE.darker());
   }

   /**
    * Note: need a public no-args constructor for ArtiSynth save/load.
    */
   public SubdivisionMesh (PolygonalMesh baseMesh) {
      this();
      myLattice.setMesh (baseMesh);
      updateSmoothedMesh();
   }

   /**
    * Returns the mesh's current pose.
    *
    * @return current pose. Should not be modified.
    */
   public RigidTransform3d getPose() {
      return new RigidTransform3d (myLattice.getMeshToWorld());
   }

   /**
    * Returns the mesh's current pose.
    *
    * @param TMW returns the current pose
    */
   public void getPose (RigidTransform3d TMW) {
      TMW.set (myLattice.getMeshToWorld());
   }

   /**
    * Sets the mesh's current pose.
    *
    * @param TMW new segment pose
    */
   public void setPose (RigidTransform3d TMW) {
      myLattice.setMeshToWorld (TMW);
      mySmoothed.setMeshToWorld (TMW);
   }

   // transform geometry methods XXX finish

   public void addTransformableDependencies (
      TransformGeometryContext context, int flags) {
   }

   public void transformGeometry(AffineTransform3dBase X) {
      TransformGeometryContext.transform (this, X, 0);
   }

   public void transformGeometry (
      GeometryTransformer gtr, TransformGeometryContext context, int flags) {

      // transform the pose
      RigidTransform3d TMW = new RigidTransform3d();
      TMW.set (getPose());
      gtr.transform (TMW);
      setPose (TMW);
   } 

   public void updateSmoothedMesh() {
      if (myLattice != null) {
         PolygonalMesh mesh = (PolygonalMesh)myLattice.getMesh();
         mesh = MeshFactory.subdivide (mesh, myFlatDivisions);
         mesh = MeshFactory.subdivideCatClark (mesh, mySubDivisions);
         mesh.setRenderProps (mySmoothed.getRenderProps());
         mySmoothed.setMesh (mesh);
         mySmoothed.setMeshToWorld (getPose());
      }
   }

   // renderable methods

   public RenderProps createRenderProps() {
      return defaultRenderProps (this);
   }

   public void render (Renderer renderer, int flags) {
   }

   public void prerender (RenderList list) {
      list.addIfVisible (myLattice);
      list.addIfVisible (mySmoothed);
   }

   public void updateBounds (Vector3d pmin, Vector3d pmax) {
      myLattice.updateBounds (pmin, pmax);
   }

   public void componentChanged (ComponentChangeEvent e) {
      super.componentChanged(e);
      if (e instanceof GeometryChangeEvent) {
         updateSmoothedMesh();
      }
   }

   // I/O method overrides to write and scan quantities not stored as
   // properties

   /**
    * {@inheritDoc}
    */
   @Override
   protected void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor) 
      throws IOException {
      super.writeItems (pw, fmt, ancestor);
      pw.println (
         "TMW=" + getPose().toString (fmt, RigidTransform3d.MATRIX_3X4_STRING));
   }

   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {

      rtok.nextToken();
      if (scanAttributeName (rtok, "TMW")) {
         RigidTransform3d TMW = new RigidTransform3d();
         TMW.scan (rtok);
         setPose (TMW);
         return true;
      }
      rtok.pushBack();
      return super.scanItem (rtok, tokens);
   }

}
