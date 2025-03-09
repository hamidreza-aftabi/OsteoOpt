package artisynth.VSP.reconstruction;

import java.awt.Color;
import java.io.*;
import java.util.*;

import artisynth.core.workspace.*;
import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.*;
import artisynth.core.femmodels.*;
import artisynth.core.materials.*;
import artisynth.core.probes.*;

import maspack.util.*;
import maspack.matrix.*;
import maspack.geometry.*;
import maspack.render.*;
import maspack.render.Renderer.*;
import maspack.properties.*;

public class OcclusalPlaneTest extends RootModel {

   public static boolean omitFromMenu = true;

   public void build (String[] args) {
      MechModel mech = new MechModel ("mech");
      addModel (mech);

      Point3d p0 = new Point3d (-50, 0, 0);
      Point3d p1 = new Point3d (0, -100, 0);
      Point3d p2 = new Point3d (50, 0, 0);
      OcclusalPlane pbody = new OcclusalPlane (p0, p1, p2);
      //pbody.setMeshSize (1.0);
      pbody.updateSurfaceMesh();

      mech.addRigidBody (pbody);
      RenderProps.setSphericalPoints (pbody, 2, Color.cyan);
      RenderProps.setFaceStyle (pbody, FaceStyle.FRONT_AND_BACK);
   }

   public StepAdjustment advance (double t0, double t1, int flags) {
      StepAdjustment sa = super.advance (t0, t1, flags);
      return sa;
   }

}
