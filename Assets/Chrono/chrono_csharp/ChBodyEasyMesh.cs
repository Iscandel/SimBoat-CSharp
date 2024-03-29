//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChBodyEasyMesh : ChBodyAuxRef {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  private bool swigCMemOwnDerived;

  internal ChBodyEasyMesh(global::System.IntPtr cPtr, bool cMemoryOwn) : base(corePINVOKE.ChBodyEasyMesh_SWIGSmartPtrUpcast(cPtr), true) {
    swigCMemOwnDerived = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChBodyEasyMesh obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwnDerived) {
          swigCMemOwnDerived = false;
          corePINVOKE.delete_ChBodyEasyMesh(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }

  public ChBodyEasyMesh(string filename, double density, bool compute_mass, bool visualize, bool collide, ChMaterialSurface material, double sphere_swept, ChCollisionModel collision_model) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_0(filename, density, compute_mass, visualize, collide, ChMaterialSurface.getCPtr(material), sphere_swept, ChCollisionModel.getCPtr(collision_model)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBodyEasyMesh(string filename, double density, bool compute_mass, bool visualize, bool collide, ChMaterialSurface material, double sphere_swept) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_1(filename, density, compute_mass, visualize, collide, ChMaterialSurface.getCPtr(material), sphere_swept), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBodyEasyMesh(string filename, double density, bool compute_mass, bool visualize, bool collide, ChMaterialSurface material) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_2(filename, density, compute_mass, visualize, collide, ChMaterialSurface.getCPtr(material)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBodyEasyMesh(string filename, double density, bool compute_mass, bool visualize, bool collide) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_3(filename, density, compute_mass, visualize, collide), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBodyEasyMesh(string filename, double density, bool compute_mass, bool visualize) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_4(filename, density, compute_mass, visualize), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBodyEasyMesh(string filename, double density, bool compute_mass) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_5(filename, density, compute_mass), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBodyEasyMesh(string filename, double density) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_6(filename, density), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBodyEasyMesh(ChTriangleMeshConnected mesh, double density, bool compute_mass, bool visualize, bool collide, ChMaterialSurface material, double sphere_swept, ChCollisionModel collision_model) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_7(ChTriangleMeshConnected.getCPtr(mesh), density, compute_mass, visualize, collide, ChMaterialSurface.getCPtr(material), sphere_swept, ChCollisionModel.getCPtr(collision_model)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBodyEasyMesh(ChTriangleMeshConnected mesh, double density, bool compute_mass, bool visualize, bool collide, ChMaterialSurface material, double sphere_swept) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_8(ChTriangleMeshConnected.getCPtr(mesh), density, compute_mass, visualize, collide, ChMaterialSurface.getCPtr(material), sphere_swept), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBodyEasyMesh(ChTriangleMeshConnected mesh, double density, bool compute_mass, bool visualize, bool collide, ChMaterialSurface material) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_9(ChTriangleMeshConnected.getCPtr(mesh), density, compute_mass, visualize, collide, ChMaterialSurface.getCPtr(material)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBodyEasyMesh(ChTriangleMeshConnected mesh, double density, bool compute_mass, bool visualize, bool collide) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_10(ChTriangleMeshConnected.getCPtr(mesh), density, compute_mass, visualize, collide), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBodyEasyMesh(ChTriangleMeshConnected mesh, double density, bool compute_mass, bool visualize) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_11(ChTriangleMeshConnected.getCPtr(mesh), density, compute_mass, visualize), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBodyEasyMesh(ChTriangleMeshConnected mesh, double density, bool compute_mass) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_12(ChTriangleMeshConnected.getCPtr(mesh), density, compute_mass), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBodyEasyMesh(ChTriangleMeshConnected mesh, double density) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_13(ChTriangleMeshConnected.getCPtr(mesh), density), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBodyEasyMesh(string filename, double density, ChMaterialSurface material, double sphere_swept, ChCollisionSystemType collision_type) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_14(filename, density, ChMaterialSurface.getCPtr(material), sphere_swept, (int)collision_type), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChBodyEasyMesh(ChTriangleMeshConnected mesh, double density, ChMaterialSurface material, double sphere_swept, ChCollisionSystemType collision_type) : this(corePINVOKE.new_ChBodyEasyMesh__SWIG_15(ChTriangleMeshConnected.getCPtr(mesh), density, ChMaterialSurface.getCPtr(material), sphere_swept, (int)collision_type), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

}
