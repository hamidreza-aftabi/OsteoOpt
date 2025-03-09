package artisynth.VSP.reconstruction;

import java.awt.Color;

import java.io.PrintWriter;
import java.io.IOException;
import java.util.Deque;
import java.util.ArrayList;

import artisynth.core.util.ScanToken;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import maspack.matrix.Vector3d;
import maspack.util.ReaderTokenizer;
import maspack.util.NumberFormat;

import maspack.matrix.*;
import maspack.util.*;
import maspack.geometry.*;
import maspack.properties.*;
import maspack.render.*;
import maspack.render.Renderer.*;

import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.*;

/**
 * Geometric description of a dental implant.
 */
public class Implant extends RigidBody {

   private static double INF = Double.POSITIVE_INFINITY;
   
   // properties:

   static double DEFAULT_UPPER_RADIUS = 2.05;
   double myUpperRadius = DEFAULT_UPPER_RADIUS;

   static double DEFAULT_LOWER_RADIUS = 2.05;
   double myLowerRadius = DEFAULT_LOWER_RADIUS;

   static double DEFAULT_LENGTH = 10;
   double myLength = DEFAULT_LENGTH;

   static double DEFAULT_TOP_EXTENSION =
      ImplantsManager.DEFAULT_OCCLUSAL_OFFSET;
   double myTopExtension = DEFAULT_TOP_EXTENSION;

   public static PropertyList myProps =
      new PropertyList (
         Implant.class, Frame.class);

   static {
      myProps.remove ("velocity");
      myProps.remove ("targetPosition");
      myProps.remove ("targetOrientation");
      myProps.remove ("targetVelocity");
      myProps.remove ("targetActivity");
      myProps.remove ("force");
      myProps.remove ("transForce");
      myProps.remove ("moment");
      myProps.remove ("externalForce");
      myProps.remove ("frameDamping");
      myProps.remove ("rotaryDamping");
      myProps.add (
         "submeshesSelectable", 
         "enables submeshes to be selected in the viewer", 
         RigidBody.DEFAULT_SUBMESHES_SELECTABLE);
      myProps.add (
         "upperRadius",
         "upper radius of the implant",
         DEFAULT_UPPER_RADIUS);
      myProps.add (
         "lowerRadius",
         "lower radius of the implant",
         DEFAULT_LOWER_RADIUS);
      myProps.add (
         "length",
         "length of the implant",
         DEFAULT_LENGTH);
      myProps.add (
         "topExtension",
         "length of extension to be rendered above the implant's top",
         DEFAULT_TOP_EXTENSION);
      
   }
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   // property accessors

   public double getUpperRadius() {
      return myUpperRadius;
   }

   public void setUpperRadius(double value) {
      if (myUpperRadius != value) {
         if (!isScanning() && getSurfaceMesh() != null) {
            updateMainMesh();
         }
         myUpperRadius = value;
      }
   }

   public double getLowerRadius() {
      return myLowerRadius;
   }

   public void setLowerRadius(double value) {
      if (myLowerRadius != value) {
         if (!isScanning() && getSurfaceMesh() != null) {
            updateMainMesh();
         }
         myLowerRadius = value;
      }
   }

   public double getLength() {
      return myLength;
   }

   public void setLength(double value) {
      if (myLength != value) {
         if (!isScanning()) {
            if (getSurfaceMesh() != null) {
               updateMainMesh();
            }
            updateTopExtension();
         }
         myLength = value;
      }
   }

   public double getTopExtension() {
      return myTopExtension;
   }

   public void setTopExtension(double value) {
      if (!isScanning() && myTopExtension != value) {
         updateTopExtension();
         myTopExtension = value;
      }
   }

   void updateTopExtension () {
      removeMeshComp ("topext");
      double topExt = getTopExtension();
      if (topExt > 0) {
         // add top extension mesh
         PolygonalMesh mesh = MeshFactory.createCylinder (
            getUpperRadius(), topExt, /*nsides=*/32);
         mesh.translate (new Vector3d (0, 0, (getLength()+topExt)/2));
         MeshComponent mcomp = addMesh (mesh);
         mcomp.setName ("topext");
         RenderProps.setFaceColor (mcomp, new Color (0.7f, 1f, 0.7f));
      }
   }

   void updateMainMesh () {
      PolygonalMesh mesh = MeshFactory.createCone(
         getUpperRadius(),
         getLowerRadius(),
         getLength(),
         /*nsides=*/32);
      setSurfaceMesh (mesh);
   }

   /**
    * No-args constructor for scan/write
    */
   public Implant() {
      setDynamic (false);
   }

   public Implant (double rupper, double rlower, double len, double topExt) {
      this();
      setDensity (1000.0);
      setSubmeshesSelectable (false);
      setUpperRadius (rupper);
      setLowerRadius (rlower);
      setLength (len);
      setTopExtension (topExt);
      updateMainMesh();
      updateTopExtension();
   }

}
