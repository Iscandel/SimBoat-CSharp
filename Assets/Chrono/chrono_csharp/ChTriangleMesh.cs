//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChTriangleMesh : ChGeometry {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  private bool swigCMemOwnDerived;

  internal ChTriangleMesh(global::System.IntPtr cPtr, bool cMemoryOwn) : base(corePINVOKE.ChTriangleMesh_SWIGSmartPtrUpcast(cPtr), true) {
    swigCMemOwnDerived = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChTriangleMesh obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwnDerived) {
          swigCMemOwnDerived = false;
          corePINVOKE.delete_ChTriangleMesh(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }

  public virtual void addTriangle(ChVectorD vertex0, ChVectorD vertex1, ChVectorD vertex2) {
    corePINVOKE.ChTriangleMesh_addTriangle__SWIG_0(swigCPtr, ChVectorD.getCPtr(vertex0), ChVectorD.getCPtr(vertex1), ChVectorD.getCPtr(vertex2));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void addTriangle(ChTriangle atriangle) {
    corePINVOKE.ChTriangleMesh_addTriangle__SWIG_1(swigCPtr, ChTriangle.getCPtr(atriangle));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual int getNumTriangles() {
    int ret = corePINVOKE.ChTriangleMesh_getNumTriangles(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public virtual ChTriangle getTriangle(int index) {
    ChTriangle ret = new ChTriangle(corePINVOKE.ChTriangleMesh_getTriangle(swigCPtr, index), true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public virtual void Clear() {
    corePINVOKE.ChTriangleMesh_Clear(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void Transform(ChVectorD displ, ChMatrix33D rotscale) {
    corePINVOKE.ChTriangleMesh_Transform__SWIG_0(swigCPtr, ChVectorD.getCPtr(displ), ChMatrix33D.getCPtr(rotscale));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void Transform(ChVectorD displ, ChQuaternionD mquat) {
    corePINVOKE.ChTriangleMesh_Transform__SWIG_1(swigCPtr, ChVectorD.getCPtr(displ), ChQuaternionD.getCPtr(mquat));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void Transform(ChVectorD displ) {
    corePINVOKE.ChTriangleMesh_Transform__SWIG_2(swigCPtr, ChVectorD.getCPtr(displ));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override ChGeometry.GeometryType GetClassType() {
    ChGeometry.GeometryType ret = (ChGeometry.GeometryType)corePINVOKE.ChTriangleMesh_GetClassType(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public override void GetBoundingBox(SWIGTYPE_p_double xmin, SWIGTYPE_p_double xmax, SWIGTYPE_p_double ymin, SWIGTYPE_p_double ymax, SWIGTYPE_p_double zmin, SWIGTYPE_p_double zmax, ChMatrix33D Rot) {
    corePINVOKE.ChTriangleMesh_GetBoundingBox__SWIG_0(swigCPtr, SWIGTYPE_p_double.getCPtr(xmin), SWIGTYPE_p_double.getCPtr(xmax), SWIGTYPE_p_double.getCPtr(ymin), SWIGTYPE_p_double.getCPtr(ymax), SWIGTYPE_p_double.getCPtr(zmin), SWIGTYPE_p_double.getCPtr(zmax), ChMatrix33D.getCPtr(Rot));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override void GetBoundingBox(SWIGTYPE_p_double xmin, SWIGTYPE_p_double xmax, SWIGTYPE_p_double ymin, SWIGTYPE_p_double ymax, SWIGTYPE_p_double zmin, SWIGTYPE_p_double zmax) {
    corePINVOKE.ChTriangleMesh_GetBoundingBox__SWIG_1(swigCPtr, SWIGTYPE_p_double.getCPtr(xmin), SWIGTYPE_p_double.getCPtr(xmax), SWIGTYPE_p_double.getCPtr(ymin), SWIGTYPE_p_double.getCPtr(ymax), SWIGTYPE_p_double.getCPtr(zmin), SWIGTYPE_p_double.getCPtr(zmax));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override void CovarianceMatrix(ChMatrix33D C) {
    corePINVOKE.ChTriangleMesh_CovarianceMatrix(swigCPtr, ChMatrix33D.getCPtr(C));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override int GetManifoldDimension() {
    int ret = corePINVOKE.ChTriangleMesh_GetManifoldDimension(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public override void ArchiveOUT(SWIGTYPE_p_ChArchiveOut marchive) {
    corePINVOKE.ChTriangleMesh_ArchiveOUT(swigCPtr, SWIGTYPE_p_ChArchiveOut.getCPtr(marchive));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override void ArchiveIN(SWIGTYPE_p_chrono__ChArchiveIn marchive) {
    corePINVOKE.ChTriangleMesh_ArchiveIN(swigCPtr, SWIGTYPE_p_chrono__ChArchiveIn.getCPtr(marchive));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

}
