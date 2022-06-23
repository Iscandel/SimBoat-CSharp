//------------------------------------------------------------------------------
// <auto-generated />
//
// This file was automatically generated by SWIG (http://www.swig.org).
// Version 4.0.2
//
// Do not make changes to this file unless you know what you are doing--modify
// the SWIG interface file instead.
//------------------------------------------------------------------------------


public class ChSurfaceNurbs : ChSurface {
  private global::System.Runtime.InteropServices.HandleRef swigCPtr;
  private bool swigCMemOwnDerived;

  internal ChSurfaceNurbs(global::System.IntPtr cPtr, bool cMemoryOwn) : base(corePINVOKE.ChSurfaceNurbs_SWIGSmartPtrUpcast(cPtr), true) {
    swigCMemOwnDerived = cMemoryOwn;
    swigCPtr = new global::System.Runtime.InteropServices.HandleRef(this, cPtr);
  }

  internal static global::System.Runtime.InteropServices.HandleRef getCPtr(ChSurfaceNurbs obj) {
    return (obj == null) ? new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero) : obj.swigCPtr;
  }

  protected override void Dispose(bool disposing) {
    lock(this) {
      if (swigCPtr.Handle != global::System.IntPtr.Zero) {
        if (swigCMemOwnDerived) {
          swigCMemOwnDerived = false;
          corePINVOKE.delete_ChSurfaceNurbs(swigCPtr);
        }
        swigCPtr = new global::System.Runtime.InteropServices.HandleRef(null, global::System.IntPtr.Zero);
      }
      base.Dispose(disposing);
    }
  }

  public SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t points {
    set {
      corePINVOKE.ChSurfaceNurbs_points_set(swigCPtr, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t.getCPtr(value));
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      global::System.IntPtr cPtr = corePINVOKE.ChSurfaceNurbs_points_get(swigCPtr);
      SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t ret = (cPtr == global::System.IntPtr.Zero) ? null : new SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t(cPtr, false);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public ChMatrixDynamicD weights {
    set {
      corePINVOKE.ChSurfaceNurbs_weights_set(swigCPtr, ChMatrixDynamicD.getCPtr(value));
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      global::System.IntPtr cPtr = corePINVOKE.ChSurfaceNurbs_weights_get(swigCPtr);
      ChMatrixDynamicD ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChMatrixDynamicD(cPtr, false);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public ChVectorDynamicD knots_u {
    set {
      corePINVOKE.ChSurfaceNurbs_knots_u_set(swigCPtr, ChVectorDynamicD.getCPtr(value));
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      global::System.IntPtr cPtr = corePINVOKE.ChSurfaceNurbs_knots_u_get(swigCPtr);
      ChVectorDynamicD ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChVectorDynamicD(cPtr, false);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public ChVectorDynamicD knots_v {
    set {
      corePINVOKE.ChSurfaceNurbs_knots_v_set(swigCPtr, ChVectorDynamicD.getCPtr(value));
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      global::System.IntPtr cPtr = corePINVOKE.ChSurfaceNurbs_knots_v_get(swigCPtr);
      ChVectorDynamicD ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChVectorDynamicD(cPtr, false);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public int p_u {
    set {
      corePINVOKE.ChSurfaceNurbs_p_u_set(swigCPtr, value);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      int ret = corePINVOKE.ChSurfaceNurbs_p_u_get(swigCPtr);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public int p_v {
    set {
      corePINVOKE.ChSurfaceNurbs_p_v_set(swigCPtr, value);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      int ret = corePINVOKE.ChSurfaceNurbs_p_v_get(swigCPtr);
      if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public ChSurfaceNurbs() : this(corePINVOKE.new_ChSurfaceNurbs__SWIG_0(), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChSurfaceNurbs(int morder_u, int morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t mpoints, ChVectorDynamicD mknots_u, ChVectorDynamicD mknots_v, ChMatrixDynamicD weights) : this(corePINVOKE.new_ChSurfaceNurbs__SWIG_1(morder_u, morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t.getCPtr(mpoints), ChVectorDynamicD.getCPtr(mknots_u), ChVectorDynamicD.getCPtr(mknots_v), ChMatrixDynamicD.getCPtr(weights)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChSurfaceNurbs(int morder_u, int morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t mpoints, ChVectorDynamicD mknots_u, ChVectorDynamicD mknots_v) : this(corePINVOKE.new_ChSurfaceNurbs__SWIG_2(morder_u, morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t.getCPtr(mpoints), ChVectorDynamicD.getCPtr(mknots_u), ChVectorDynamicD.getCPtr(mknots_v)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChSurfaceNurbs(int morder_u, int morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t mpoints, ChVectorDynamicD mknots_u) : this(corePINVOKE.new_ChSurfaceNurbs__SWIG_3(morder_u, morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t.getCPtr(mpoints), ChVectorDynamicD.getCPtr(mknots_u)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChSurfaceNurbs(int morder_u, int morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t mpoints) : this(corePINVOKE.new_ChSurfaceNurbs__SWIG_4(morder_u, morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t.getCPtr(mpoints)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public ChSurfaceNurbs(ChSurfaceNurbs source) : this(corePINVOKE.new_ChSurfaceNurbs__SWIG_5(ChSurfaceNurbs.getCPtr(source)), true) {
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override ChGeometry Clone() {
    global::System.IntPtr cPtr = corePINVOKE.ChSurfaceNurbs_Clone(swigCPtr);
    ChSurfaceNurbs ret = (cPtr == global::System.IntPtr.Zero) ? null : new ChSurfaceNurbs(cPtr, true);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public override void Evaluate(ChVectorD pos, double parU, double parV) {
    corePINVOKE.ChSurfaceNurbs_Evaluate(swigCPtr, ChVectorD.getCPtr(pos), parU, parV);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public double ComputeUfromKnotU(double u) {
    double ret = corePINVOKE.ChSurfaceNurbs_ComputeUfromKnotU(swigCPtr, u);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double ComputeKnotUfromU(double U) {
    double ret = corePINVOKE.ChSurfaceNurbs_ComputeKnotUfromU(swigCPtr, U);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double ComputeVfromKnotV(double v) {
    double ret = corePINVOKE.ChSurfaceNurbs_ComputeVfromKnotV(swigCPtr, v);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public double ComputeKnotVfromV(double V) {
    double ret = corePINVOKE.ChSurfaceNurbs_ComputeKnotVfromV(swigCPtr, V);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t Points() {
    SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t ret = new SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t(corePINVOKE.ChSurfaceNurbs_Points(swigCPtr), false);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChMatrixDynamicD Weights() {
    ChMatrixDynamicD ret = new ChMatrixDynamicD(corePINVOKE.ChSurfaceNurbs_Weights(swigCPtr), false);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorDynamicD Knots_u() {
    ChVectorDynamicD ret = new ChVectorDynamicD(corePINVOKE.ChSurfaceNurbs_Knots_u(swigCPtr), false);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public ChVectorDynamicD Knots_v() {
    ChVectorDynamicD ret = new ChVectorDynamicD(corePINVOKE.ChSurfaceNurbs_Knots_v(swigCPtr), false);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public int GetOrder_u() {
    int ret = corePINVOKE.ChSurfaceNurbs_GetOrder_u(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public int GetOrder_v() {
    int ret = corePINVOKE.ChSurfaceNurbs_GetOrder_v(swigCPtr);
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public virtual void SetupData(int morder_u, int morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t mpoints, ChVectorDynamicD mknots_u, ChVectorDynamicD mknots_v, ChMatrixDynamicD weights) {
    corePINVOKE.ChSurfaceNurbs_SetupData__SWIG_0(swigCPtr, morder_u, morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t.getCPtr(mpoints), ChVectorDynamicD.getCPtr(mknots_u), ChVectorDynamicD.getCPtr(mknots_v), ChMatrixDynamicD.getCPtr(weights));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void SetupData(int morder_u, int morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t mpoints, ChVectorDynamicD mknots_u, ChVectorDynamicD mknots_v) {
    corePINVOKE.ChSurfaceNurbs_SetupData__SWIG_1(swigCPtr, morder_u, morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t.getCPtr(mpoints), ChVectorDynamicD.getCPtr(mknots_u), ChVectorDynamicD.getCPtr(mknots_v));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void SetupData(int morder_u, int morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t mpoints, ChVectorDynamicD mknots_u) {
    corePINVOKE.ChSurfaceNurbs_SetupData__SWIG_2(swigCPtr, morder_u, morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t.getCPtr(mpoints), ChVectorDynamicD.getCPtr(mknots_u));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public virtual void SetupData(int morder_u, int morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t mpoints) {
    corePINVOKE.ChSurfaceNurbs_SetupData__SWIG_3(swigCPtr, morder_u, morder_v, SWIGTYPE_p_chrono__ChMatrixDynamicT_chrono__ChVectorT_double_t_t.getCPtr(mpoints));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override void ArchiveOUT(SWIGTYPE_p_ChArchiveOut marchive) {
    corePINVOKE.ChSurfaceNurbs_ArchiveOUT(swigCPtr, SWIGTYPE_p_ChArchiveOut.getCPtr(marchive));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

  public override void ArchiveIN(SWIGTYPE_p_chrono__ChArchiveIn marchive) {
    corePINVOKE.ChSurfaceNurbs_ArchiveIN(swigCPtr, SWIGTYPE_p_chrono__ChArchiveIn.getCPtr(marchive));
    if (corePINVOKE.SWIGPendingException.Pending) throw corePINVOKE.SWIGPendingException.Retrieve();
  }

}