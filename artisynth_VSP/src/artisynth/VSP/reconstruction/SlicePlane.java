package artisynth.VSP.reconstruction;

import java.util.*;

import maspack.matrix.*;
import maspack.properties.*;
import maspack.image.dicom.*;
import maspack.geometry.*;
import maspack.collision.*;
import maspack.render.*;
import maspack.render.Renderer.*;
import artisynth.core.renderables.*;
import artisynth.core.mechmodels.*;

import java.io.PrintWriter;
import java.io.IOException;
import java.util.Deque;
import java.util.ArrayList;

import artisynth.core.util.ScanToken;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.ScanWriteUtils;
import maspack.matrix.Vector3d;
import maspack.util.ReaderTokenizer;
import maspack.util.NumberFormat;

public class SlicePlane extends DicomPlaneViewer {

   ArrayList<IntersectionContour> myContours;
   ArrayList<RigidBody> myContourBodies = new ArrayList<>();

   static boolean DEFAULT_DRAW_CONTOURS = true;
   boolean myDrawContours = DEFAULT_DRAW_CONTOURS;

   public static PropertyList myProps = new PropertyList (
      SlicePlane.class, DicomPlaneViewer.class);

   static private RenderProps defaultRenderProps = new RenderProps ();

   static {
      myProps.add (
         "drawContours",
         "draw intersection contours with bodies", DEFAULT_DRAW_CONTOURS);
   }

   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   /**
    * Creates an empty slice plane. Used by scan methods.
    */
   public SlicePlane () {
      super();
   }

   public SlicePlane (
      String name, DicomViewer dviewer, PolygonalMesh templateMesh,
      RigidTransform3d TPW, RigidTransform3d TVI) {
      super (name, dviewer, templateMesh, TPW, TVI);
   }

   public void setDrawContours (boolean enable) {
      myDrawContours = enable;
   }

   public boolean getDrawContours () {
      return myDrawContours;
   }

   public void addContourBody (RigidBody body) {
      myContourBodies.add (body);
   }

   public boolean removeContourBody (RigidBody body) {
      return myContourBodies.remove (body);
   }

   public void removeAllContourBodies() {
      myContourBodies.clear();
   }

   public int numContourBodies() {
      return myContourBodies.size();
   }

   @Override
   public void prerender (RenderList list) {
      super.prerender (list);
      if (myDrawContours) {
         ArrayList<IntersectionContour> clist = new ArrayList<>();
         PolygonalMesh plane = MeshFactory.createRectangle (300, 300, false);
         plane.setMeshToWorld (getPose());
         SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();
         for (RigidBody body : myContourBodies) {
            ArrayList<IntersectionContour> contours =
               intersector.findContours (body.getSurfaceMesh(), plane);
            for (IntersectionContour c : contours) {
               if (c.isClosed()) {
                  clist.add (c);
               }
            }
         }
         myContours = clist;
      }
      else {
         myContours = null;
      }
   }   

   @Override
   public void render (Renderer renderer, int flags) {
      ArrayList<IntersectionContour> contours = myContours;
      if (contours != null) {
         renderer.setLineWidth (myRenderProps.getLineWidth());
         renderer.setColor (myRenderProps.getLineColor());
         for (IntersectionContour c : contours) {
            if (c.isClosed()) {
               renderer.beginDraw (DrawMode.LINE_LOOP);
            }
            else {
               renderer.beginDraw (DrawMode.LINE_STRIP);
            }
            for (IntersectionPoint p : c) {
               renderer.addVertex ((float)p.x, (float)p.y, (float)p.z);
            }
            renderer.endDraw();
         }           
      }
      super.render (renderer, flags);  
   }

   // --- I/O methods ---
   
   protected void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor)
      throws IOException {

      pw.print ("contourBodies=");
      ScanWriteUtils.writeBracketedReferences (pw, myContourBodies, ancestor);

      // if (myRefComp != null) {
      //    pw.println (
      //       "refComp="+ComponentUtils.getWritePathName (ancestor,myRefComp));
      // }
      // pw.print ("vec=");
      // myVec.write (pw, fmt, /*withBrackets=*/true);
      // pw.println ("");
      // pw.print ("scalar=" + myScalar);
      // pw.print ("string=\"" + myString + "\"");
      super.writeItems (pw, fmt, ancestor);
   }

   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {

      rtok.nextToken();
      // if (scanAndStoreReference (rtok, "refComp", tokens)) {
      //    return true;
      // }
      if (ScanWriteUtils.scanAndStoreReferences (
                  rtok, "contourBodies", tokens) != -1) {
         return true;
      }
      // else if (scanAttributeName (rtok, "vec")) {
      //    myVec.scan (rtok);
      //    return true;
      // }
      // else if (scanAttributeName (rtok, "scalar")) {
      //    myScalar = rtok.scanNumber();
      //    return true;
      // }
      // else if (scanAttributeName (rtok, "string")) {
      //    myString = rtok.scanQuotedString('"');
      //    return true;
      // }
      rtok.pushBack();
      return super.scanItem (rtok, tokens);
   }

   protected boolean postscanItem (
      Deque<ScanToken> tokens, CompositeComponent ancestor) throws IOException {
      
      // if (postscanAttributeName (tokens, "refComp")) {
      //    myRefComp = 
      //       postscanReference (tokens, ModelComponent.class, ancestor);
      //    return true;
      // }
      if (postscanAttributeName (tokens, "contourBodies")) {
         myContourBodies.clear();
         ScanWriteUtils.postscanReferences (
            tokens, myContourBodies, RigidBody.class, ancestor);
         return true;
      }
      return super.postscanItem (tokens, ancestor);
   }


}
